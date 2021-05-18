# BSD 3-Clause License
#
# Copyright (c) 2018, Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from typing import Type, Union, Dict, Optional, Any, cast, Sequence, Mapping
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

import scipy.signal as signal

from bag.simulation.measure import MeasurementManager, MeasInfo
from bag.simulation.core import TestbenchManager
from bag.simulation.data import SimNetlistInfo, netlist_info_from_dict
from bag.simulation.cache import SimulationDB, DesignInstance, SimResults, MeasureResult
from bag.design.module import Module
from bag.concurrent.util import GatherHelper

from bag3_testbenches.measurement.ac import ACTB


class HighpassACMeas(MeasurementManager):
    def get_sim_info(self, sim_db: SimulationDB, dut: DesignInstance, cur_info: MeasInfo):
        raise NotImplementedError

    def initialize(self, sim_db: SimulationDB, dut: DesignInstance):
        raise NotImplementedError

    def process_output(self, cur_info: MeasInfo, sim_results: Union[SimResults, MeasureResult]):
        raise NotImplementedError

    async def async_measure_performance(self, name: str, sim_dir: Path, sim_db: SimulationDB,
                                        dut: Optional[DesignInstance]) -> Mapping[str, Any]:
        """Skip existing methods and just do everything here"""
        helper = GatherHelper()
        for idx in range(4):
            helper.append(self.async_meas_case(name, sim_dir, sim_db, dut, idx))

        meas_results = await helper.gather_err()
        ans = self.compute_passives(meas_results)

        tf_results = await self.async_meas_tf(name, sim_dir, sim_db, dut)
        plt.semilogx(2*np.pi*tf_results['freq'], 20*np.log10(tf_results['tf']))

        r = ans['r']
        cc = ans['cc']
        tf_calc = signal.TransferFunction([r*cc, 0], [r*cc, 1])
        w, mag, _ = signal.bode(tf_calc, 2 * np.pi * tf_results['freq'])
        plt.semilogx(w, mag)
        plt.grid()

        plt.show()

        return ans

    @staticmethod
    def compute_passives(meas_results: Sequence[Mapping[str, Any]]) -> Mapping[str, Any]:
        freq0 = meas_results[0]['freq']
        freq1 = meas_results[1]['freq']
        freq2 = meas_results[2]['freq']
        freq3 = meas_results[3]['freq']
        assert np.isclose(freq0, freq1).all()
        assert np.isclose(freq0, freq2).all()
        assert np.isclose(freq0, freq3).all()
        # breakpoint()

        # plt.loglog(freq0, np.abs(meas_results[0]['vtest']))
        # plt.loglog(freq0, np.abs(meas_results[1]['vtest']))
        # plt.loglog(freq0, np.abs(meas_results[2]['vtest']))
        #
        # plt.show()

        # vm0 = (zc * zpm) / (zc + zpp + zpm)
        # vp0 = - (zc * zpp) / (zc + zpp + zpm)
        vm0 = meas_results[0]['vm']
        vp0 = meas_results[0]['vp']

        # vm1 = - (zpp * zpm) / (zc + zpp + zpm)
        # vp1 = - ((zc + zpm) * zpp) / (zc + zpp + zpm)
        vm1 = meas_results[1]['vm']
        vp1 = meas_results[1]['vp']

        # --- Find zc, zpp, zpm using vm0, vp0, vm1 --- #
        # - vp0 / vm0 = zpp / zpm = const_a  ==> zpp = const_a * zpm
        const_a = - vp0 / vm0
        # vp0 / vm1 = zc / zpm = const_b  ==> zc = const_b * zpm
        const_b = vp0 / vm1

        # vp0 = - (const_b * const_a * zpm) / (const_b + const_a + 1)
        zpm = - vp0 * (const_b + const_a + 1) / (const_b * const_a)
        zpp = const_a * zpm
        zc = const_b * zpm

        plt.loglog(freq0, np.abs(zpm))
        plt.loglog(freq0, np.abs(zpp))
        plt.loglog(freq0, np.abs(zc))
        plt.grid()
        plt.show()
        breakpoint()

        cc = estimate_cap(freq0, zc)
        cpi = estimate_cap(freq0, zpp)
        cpo_cpb = estimate_cap(freq0, zpm)

        #
        # # --- Verify vp1 is consistent --- #
        vp1_calc = - ((zc + zpm) * zpp) / (zc + zpp + zpm)
        if not np.isclose(vp1, vp1_calc, rtol=1e-3).all():
        # if True:
            plt.loglog(freq0, np.abs(vp1), label='measured')
            plt.loglog(freq0, np.abs(vp1_calc), 'g--', label='calculated')
            plt.xlabel('Frequency (in Hz)')
            plt.ylabel('Value')
            plt.legend()
            plt.show()

        # vm0 = (zc * zpm) / (zc + zpp + zpm)
        # vp0 = - (zc * zpp) / (zc + zpp + zpm)
        vm0 = meas_results[2]['vm']
        vp0 = meas_results[2]['vp']

        # vm1 = - (zpp * zpm) / (zc + zpp + zpm)
        # vp1 = - ((zc + zpm) * zpp) / (zc + zpp + zpm)
        vm1 = meas_results[3]['vm']
        vp1 = meas_results[3]['vp']

        # --- Find zc, zpp, zpm using vm0, vp0, vm1 --- #
        # - vp0 / vm0 = zpp / zpm = const_a  ==> zpp = const_a * zpm
        const_a = - vp0 / vm0
        # vp0 / vm1 = zc / zpm = const_b  ==> zc = const_b * zpm
        const_b = vp0 / vm1

        # vp0 = - (const_b * const_a * zpm) / (const_b + const_a + 1)
        zpm = - vp0 * (const_b + const_a + 1) / (const_b * const_a)
        zpp = const_a * zpm
        zc = const_b * zpm

        plt.loglog(freq0, np.abs(zpm))
        plt.loglog(freq0, np.abs(zpp))
        plt.loglog(freq0, np.abs(zc))
        plt.grid()
        plt.show()
        breakpoint()

        r = np.mean(zc)
        cpb = estimate_cap(freq0, zpp)
        cpi_cpo = estimate_cap(freq0, zpm)
        cpo = cpo_cpb - cpb
        assert np.isclose(cpi_cpo - cpo, cpi).all()

        # # --- Verify vp1 is consistent --- #
        vp1_calc = - ((zc + zpm) * zpp) / (zc + zpp + zpm)
        if not np.isclose(vp1, vp1_calc, rtol=1e-3).all():
        # if True:
            plt.loglog(freq0, np.abs(vp1), label='measured')
            plt.loglog(freq0, np.abs(vp1_calc), 'g--', label='calculated')
            plt.xlabel('Frequency (in Hz)')
            plt.ylabel('Value')
            plt.legend()
            plt.show()

        return dict(
            r=r,
            cc=cc,
            cpi=cpi,
            cpo=cpo,
            cpb=cpb,
        )

    async def async_meas_tf(self, name: str, sim_dir: Path, sim_db: SimulationDB, dut: Optional[DesignInstance]) -> Dict[str, Any]:

        src_list = [dict(lib='analogLib', type='vsin', value='1m', conns={'PLUS': 'inp', 'MINUS': 'VSS'})]
        dut_conns = dict(biasp='VSS', outp='outp', inp='inp', biasn='VSS', outn='VSS', inn='inn')

        tbm_specs = dict(
            **self.specs['tbm_specs'],
            save_outputs=['inp', 'outp'],
            src_list=src_list,
            sim_envs=self.specs['sim_envs'],
        )
        tbm = cast(ACTB, self.make_tbm(ACTB, tbm_specs))
        tbm_name = name
        tb_params = dict(
            extracted=self.specs['tbm_specs'].get('extracted', True),
            dut_conns=dut_conns,
        )
        sim_results = await sim_db.async_simulate_tbm_obj(tbm_name, sim_dir / tbm_name, dut, tbm,
                                                          tb_params=tb_params)
        data = sim_results.data
        inp = np.squeeze(data['inp'])
        outp = np.squeeze(data['outp'])
        tf = outp / inp

        return dict(
            freq=data['freq'],
            inp=inp,
            outp=outp,
            tf=tf
        )

    async def async_meas_case(self, name: str, sim_dir: Path, sim_db: SimulationDB, dut: Optional[DesignInstance],
                              case_idx: int) -> Dict[str, Any]:
        if case_idx == 0:
            src_list = [dict(lib='analogLib', type='isin', value='1', conns={'PLUS': 'itestp', 'MINUS': 'itestm'})]
            dut_conns = dict(biasp='itestm', outp='itestm', inp='itestp', biasn='VSS', outn='VSS', inn='inn')
        elif case_idx == 1:
            src_list = [dict(lib='analogLib', type='isin', value='1', conns={'PLUS': 'itestp', 'MINUS': 'VSS'})]
            dut_conns = dict(biasp='itestm', outp='itestm', inp='itestp', biasn='VSS', outn='VSS', inn='inn')
        elif case_idx == 2:
            src_list = [dict(lib='analogLib', type='isin', value='1', conns={'PLUS': 'itestp', 'MINUS': 'itestm'})]
            dut_conns = dict(biasp='itestp', outp='itestm', inp='itestm', biasn='VSS', outn='VSS', inn='inn')
        elif case_idx == 3:
            src_list = [dict(lib='analogLib', type='isin', value='1', conns={'PLUS': 'itestp', 'MINUS': 'VSS'})]
            dut_conns = dict(biasp='itestp', outp='itestm', inp='itestm', biasn='VSS', outn='VSS', inn='inn')
        else:
            raise ValueError(f'Invalid case_idx={case_idx}')

        tbm_specs = dict(
            **self.specs['tbm_specs'],
            save_outputs=['itestp', 'itestm'],
            src_list=src_list,
            sim_envs=self.specs['sim_envs'],
        )
        tbm = cast(ACTB, self.make_tbm(ACTB, tbm_specs))
        tbm_name = f'{name}_{case_idx}'
        tb_params = dict(
            extracted=self.specs['tbm_specs'].get('extracted', True),
            dut_conns=dut_conns,
        )
        sim_results = await sim_db.async_simulate_tbm_obj(tbm_name, sim_dir / tbm_name, dut, tbm,
                                                          tb_params=tb_params)
        data = sim_results.data
        return dict(
            freq=data['freq'],
            vp=np.squeeze(data['itestp']),
            vm=np.squeeze(data['itestm']),
        )


def estimate_cap(freq: np.ndarray, zc: np.ndarray) -> float:
    fit = np.polyfit(2 * np.pi * freq, - 1 / np.imag(zc), 1)
    return fit[0]
