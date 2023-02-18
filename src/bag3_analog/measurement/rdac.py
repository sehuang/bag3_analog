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

from typing import Any, Mapping, Optional, Sequence
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

from bag.simulation.cache import SimulationDB, DesignInstance
from bag.simulation.measure import MeasurementManager
from bag.simulation.data import SimData
from bag.concurrent.util import GatherHelper
from bag.math.interpolate import LinearInterpolator

from bag3_testbenches.measurement.tran.digital import DigitalTranTB
from bag3_testbenches.measurement.digital.util import setup_digital_tran


class RDACMeas(MeasurementManager):
    async def async_measure_performance(self, name: str, sim_dir: Path, sim_db: SimulationDB,
                                        dut: Optional[DesignInstance],
                                        harnesses: Optional[Sequence[DesignInstance]] = None) -> Mapping[str, Any]:
        helper = GatherHelper()
        sim_envs = self.specs['sim_envs']
        for sim_env in sim_envs:
            helper.append(self.async_meas_pvt(name, sim_dir / sim_env, sim_db, dut, sim_env))

        meas_results = await helper.gather_err()
        results = {}
        for idx, sim_env in enumerate(sim_envs):
            results[sim_env] = meas_results[idx]
        self.plot_results(results)
        return results

    async def async_meas_pvt(self, name: str, sim_dir: Path, sim_db: SimulationDB, dut: Optional[DesignInstance],
                             pvt: str) -> Mapping[str, Any]:
        num_sel: int = self.specs['num_sel']

        # create load
        load_list = [dict(pin='out', type='cap', value='c_load')]

        # create clocks and inputs
        pulse_list = []
        for idx in range(num_sel):
            t_per = f't_per*{1 << idx}'
            t_pw = 't_per/2' if idx == 0 else f't_per*{1 << (idx - 1)}'
            pulse_list.append(dict(pin=f'sel<{idx}>', tper=t_per, tpw=t_pw, trf='t_rf', pos=False))

        suf = f'<{num_sel - 1}:0>'
        tb_params = dict(
            pulse_list=pulse_list,
            load_list=load_list,
            sim_envs=[pvt],
            save_outputs=[f'sel{suf}', 'out']
        )
        tbm_specs, tb_params = setup_digital_tran(self.specs, dut, **tb_params)
        tbm = self.make_tbm(DigitalTranTB, tbm_specs)
        tbm.sim_params['t_sim'] = f't_rst+t_per*{1 << (num_sel - 1)}'
        sim_results = await sim_db.async_simulate_tbm_obj(name, sim_dir, dut, tbm, tb_params)
        results = self.process_data(sim_results.data, tbm.sim_params['t_per'], tbm.sim_params['t_rst'], num_sel)
        return results

    @classmethod
    def process_data(cls, sim_data: SimData, t_per: float, t_rst: float, num_sel: int) -> Mapping[str, Any]:
        time = sim_data['time']
        out = np.squeeze(sim_data['out'])
        out_interp = LinearInterpolator([time], out, [t_per / 10])

        num_points = 1 << num_sel
        t_per2 = t_per / 2
        time_points = np.linspace(t_rst + t_per2, t_rst + t_per2 * num_points, num_points)
        out_values = out_interp(time_points)

        out_inc = out_values[1:] - out_values[:-1]
        out_inc_mean = np.mean(out_inc)
        out_dnl = (out_inc - out_inc_mean) / out_inc_mean

        return dict(
            time=time,
            out=out,
            out_dnl=out_dnl,
        )

    @classmethod
    def plot_results(cls, results: Mapping[str, Any]) -> None:
        fig, [ax0, ax1] = plt.subplots(1, 2)
        ax0.set(xlabel='Time (ns)', ylabel='RDAC output (V)')
        ax1.set(xlabel='Code', ylabel='DNL')
        ax0.grid()
        ax1.grid()
        x_dnl = None
        for pvt, data in results.items():
            ax0.plot(data['time'] * 1e9, data['out'], label=pvt)
            out_dnl = data['out_dnl']
            if x_dnl is None:
                num_dnl = len(out_dnl)
                x_dnl = np.linspace(1, num_dnl, num_dnl)
            ax1.stem(x_dnl, out_dnl, label=pvt, use_line_collection=True)
        ax0.legend()
        ax1.legend()
        plt.tight_layout()
        plt.show()
