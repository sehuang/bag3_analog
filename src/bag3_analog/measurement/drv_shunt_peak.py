from __future__ import annotations

from typing import Any, Mapping, Optional, Union, Sequence
from pathlib import Path
from shutil import copy
import matplotlib.pyplot as plt
import numpy as np

from bag.simulation.cache import SimulationDB, DesignInstance, SimResults
from bag.simulation.measure import MeasurementManager
from bag.simulation.data import SimData
from bag.concurrent.util import GatherHelper
from bag.math import float_to_si_string

from bag3_testbenches.measurement.ac.base import ACTB
from bag3_testbenches.measurement.tran.digital import DigitalTranTB
from bag3_testbenches.measurement.tran.eye import EyeAnalysis, EyeResults
from bag3_testbenches.measurement.digital.util import setup_digital_tran


class DrvShuntPeakACMeas(MeasurementManager):
    async def async_measure_performance(self, name: str, sim_dir: Path, sim_db: SimulationDB,
                                        dut: Optional[DesignInstance],
                                        harnesses: Optional[Sequence[DesignInstance]] = None) -> Mapping[str, Any]:
        helper = GatherHelper()
        sim_envs: Sequence[str] = self.specs['sim_envs']
        sweep_cases: Sequence[Mapping[str, Any]] = self.specs['sweep_cases']
        for sim_env in sim_envs:
            for swidx, sweep_case in enumerate(sweep_cases):
                helper.append(self.async_meas_pvt_case(name, sim_dir / sim_env / f'{swidx}', sim_db, dut, harnesses,
                                                       sim_env, sweep_case))

        meas_results = await helper.gather_err()
        results = {}
        idx = 0
        for sim_env in sim_envs:
            results[sim_env] = {}
            for swidx, sweep_case in enumerate(sweep_cases):
                results[sim_env][swidx] = {'data': meas_results[idx], 'sweep_case': sweep_case}
                idx += 1

        self.plot_results(results)
        return results

    async def async_meas_pvt_case(self, name: str, sim_dir: Path, sim_db: SimulationDB, dut: Optional[DesignInstance],
                                  harnesses: Optional[Sequence[DesignInstance]], pvt: str,
                                  sweep_case: Mapping[str, Any]) -> SimData:
        # outputs to be saved
        save_outputs = ['i_outp', 'i_outm', 'v_inp', 'v_inm', 'v_tail_g', 'v_tail', 'ind_p', 'ind_m']

        # create loads
        load_list = [dict(pin='i_outp', type='cap', value='c_load'),
                     dict(pin='i_outp', nin='VDD', type='res', value='r_term'),
                     dict(pin='i_outm', type='cap', value='c_load'),
                     dict(pin='i_outm', nin='VDD', type='res', value='r_term')]

        # inductors
        ind_specs: Optional[Mapping[str, Any]] = self.specs.get('ind_specs')
        if ind_specs:
            ideal: bool = ind_specs['ideal']
            if ideal:
                load_list.extend([dict(pin='ind_p', nin='VDD', type='ind', value='l_shunt'),
                                  dict(pin='ind_m', nin='VDD', type='ind', value='l_shunt')])
            else:
                sp_file_specs: Mapping[str, Any] = ind_specs['sp_file_specs']
                sim_dir.mkdir(parents=True, exist_ok=True)
                for key, _specs in sp_file_specs.items():
                    file_name: Path = Path(_specs['file_name'])
                    _num = file_name.suffix[2:-1]
                    ind_sp = f'{key}.s{_num}p'
                    copy(Path(file_name), sim_dir / ind_sp)
                    conns: Sequence[Mapping[str, str]] = _specs['conns']
                    for _idx, _conns in enumerate(conns):
                        load_list.append(dict(conns=_conns, type=f'n{_num}port', value=ind_sp,
                                              name=f'NPORT_{key}_{_idx}'))

        # create sources
        load_list.extend([dict(pin='v_tail_g', type='vdc', value='v_tail_g'),
                          dict(pin='v_inp', type='vdc', value={'vdc': 'v_incm', 'acm': 0.5}),
                          dict(pin='v_inm', type='vdc', value={'vdc': 'v_incm', 'acm': -0.5})])

        # setup harness
        if harnesses:
            harnesses_list: Sequence[Mapping[str, Any]] = self.specs['harnesses_list']
        else:
            harnesses_list = []

        tb_params = dict(
            load_list=load_list,
            harnesses_list=harnesses_list,
            sim_envs=[pvt],
            ac_options={'oppoint': 'logfile'},
            save_outputs=save_outputs,
        )
        tbm_specs, tb_params = setup_digital_tran(self.specs, dut, **tb_params)
        tbm = self.make_tbm(ACTB, tbm_specs)
        for key, val in sweep_case.items():
            tbm.sim_params[key] = val
        sim_results = await sim_db.async_simulate_tbm_obj(f'{name}_{pvt}_{get_label(sweep_case)}', sim_dir, dut, tbm,
                                                          tb_params, harnesses=harnesses)
        return sim_results.data

    @staticmethod
    def plot_results(results: Mapping[str, Any]) -> None:
        fig, ax_list = plt.subplots(1, len(results.keys()))
        if not isinstance(ax_list, np.ndarray):
            ax_list = np.array([ax_list])

        aidx = 0
        for sim_env, sweeps in results.items():
            ax = ax_list[aidx]
            for swidx, ans in results[sim_env].items():
                ac_data = ans['data']
                freq = ac_data['freq']
                out_d = ac_data['i_outp'][0] - ac_data['i_outm'][0]
                sweep_case: Mapping[str, Any] = ans['sweep_case']
                ax.semilogx(freq, 20 * np.log10(np.abs(out_d)), label=get_label(sweep_case))
            ax.legend()
            ax.grid()
            ax.set_xlabel('Frequency (Hz)')
            ax.set_ylabel('AC gain (dB)')
            ax.set_title(sim_env)
            aidx += 1

        plt.tight_layout()
        plt.show()


def get_label(sweep_case: Optional[Mapping[str, Any]]) -> str:
    if sweep_case is None:
        return ''
    _label_list = [f'{key}_{float_to_si_string(val)}' for key, val in sweep_case.items()]
    return '_'.join(_label_list)


class DrvShuntPeakTranMeas(MeasurementManager):
    async def async_measure_performance(self, name: str, sim_dir: Path, sim_db: SimulationDB,
                                        dut: Optional[DesignInstance],
                                        harnesses: Optional[Sequence[DesignInstance]] = None) -> Mapping[str, Any]:
        helper = GatherHelper()
        sim_envs: Sequence[str] = self.specs['sim_envs']
        v_incm_swp: Union[np.ndarray, Sequence[float]] = self.specs['v_incm_swp']
        if isinstance(v_incm_swp, Sequence):
            v_incm_swp = np.array(v_incm_swp)
        v_tail_g_swp: Union[np.ndarray, Sequence[float]] = self.specs['v_tail_g_swp']
        if isinstance(v_tail_g_swp, Sequence):
            v_tail_g_swp = np.array(v_tail_g_swp)
        for sim_env in sim_envs:
            for v_incm in v_incm_swp:
                for v_tail_g in v_tail_g_swp:
                    _dir = f'v_incm_{float_to_si_string(v_incm)}_v_tail_g_{float_to_si_string(v_tail_g)}'
                    helper.append(self.async_meas_pvt_case(name, sim_dir / sim_env / _dir, sim_db, dut, harnesses,
                                                           sim_env, v_incm, v_tail_g))

        meas_results = await helper.gather_err()

        # arrange meas_results in "results" for Designer and "plot_results" for plotting
        results = {'sim_envs': np.array(sim_envs), 'v_incm': v_incm_swp, 'v_tail_g': v_tail_g_swp}
        plot_results = {'sim_envs': np.array(sim_envs), 'v_incm': v_incm_swp, 'v_tail_g': v_tail_g_swp}
        len_sim_envs = len(sim_envs)
        len_v_incm = len(v_incm_swp)
        len_v_tail_g = len(v_tail_g_swp)
        results.update({'width': np.empty((len_sim_envs, len_v_incm, len_v_tail_g), dtype=float),
                        'height': np.empty((len_sim_envs, len_v_incm, len_v_tail_g), dtype=float),
                        'i_avg': np.empty((len_sim_envs, len_v_incm, len_v_tail_g), dtype=float)})
        plot_results['eye'] = [[] for _ in sim_envs]
        idx = 0
        for jdx, _ in enumerate(sim_envs):
            for kdx, _ in enumerate(v_incm_swp):
                for ldx, _ in enumerate(v_tail_g_swp):
                    ans = meas_results[idx]
                    eye_res: EyeResults = ans['eye']
                    results['width'][jdx, kdx, ldx] = eye_res.width
                    results['height'][jdx, kdx, ldx] = eye_res.height
                    results['i_avg'][jdx, kdx, ldx] = ans['i_avg']
                    plot_results['eye'][jdx].append(eye_res)
                    idx += 1

        plot_eye: bool = self.specs.get('plot_eye', False)
        if plot_eye:
            self.plot_results(plot_results)
        return results

    async def async_meas_pvt_case(self, name: str, sim_dir: Path, sim_db: SimulationDB, dut: Optional[DesignInstance],
                                  harnesses: Optional[Sequence[DesignInstance]], pvt: str, v_incm: float,
                                  v_tail_g: float) -> Mapping[str, Any]:
        # outputs to be saved
        save_outputs = ['i_outp', 'i_outm', 'v_inp', 'v_inm', 'v_tail_g', 'v_tail', 'ind_p', 'ind_m', 'VDC_VDD:p']

        # create loads
        load_list = [dict(pin='i_outp', type='cap', value='c_load'),
                     dict(pin='i_outp', nin='VDD', type='res', value='r_term'),
                     dict(pin='i_outm', type='cap', value='c_load'),
                     dict(pin='i_outm', nin='VDD', type='res', value='r_term')]

        # inductors
        ind_specs: Optional[Mapping[str, Any]] = self.specs.get('ind_specs')
        if ind_specs:
            ideal: bool = ind_specs['ideal']
            if ideal:
                load_list.extend([dict(pin='ind_p', nin='VDD', type='ind', value='l_shunt'),
                                  dict(pin='ind_m', nin='VDD', type='ind', value='l_shunt')])
            else:
                sp_file_specs: Mapping[str, Any] = ind_specs['sp_file_specs']
                sim_dir.mkdir(parents=True, exist_ok=True)
                for key, _specs in sp_file_specs.items():
                    file_name: Path = Path(_specs['file_name'])
                    _num = file_name.suffix[2:-1]
                    ind_sp = f'{key}.s{_num}p'
                    copy(Path(file_name), sim_dir / ind_sp)
                    conns: Sequence[Mapping[str, str]] = _specs['conns']
                    for _idx, _conns in enumerate(conns):
                        load_list.append(dict(conns=_conns, type=f'n{_num}port', value=ind_sp,
                                              name=f'NPORT_{key}_{_idx}'))

        # create sources
        load_list.extend([dict(pin='v_tail_g', type='vdc', value='v_tail_g'),
                          dict(conns={'vout': 'prbs_data'}, lib='ahdlLib', type='rand_bit_stream',
                               value={'tperiod': 't_per', 'vlogic_high': 'v_hi', 'vlogic_low': 'v_lo',
                                      'tdel': 't_delay', 'trise': 't_rf', 'tfall': 't_rf', 'seed': 101}),
                          dict(conns={'d': 'prbs_data', 'p': 'v_inp', 'n': 'v_inm', 'c': 'v_incm'}, type='ideal_balun',
                               value={}),
                          dict(pin='v_incm', type='vdc', value='v_incm')])

        # setup harness
        if harnesses:
            harnesses_list: Sequence[Mapping[str, Any]] = self.specs['harnesses_list']
        else:
            harnesses_list = []

        tb_params = dict(
            load_list=load_list,
            harnesses_list=harnesses_list,
            sim_envs=[pvt],
            tran_options={'errpreset': 'conservative', 'noisefmax': 'fmax_noise'},
            save_outputs=save_outputs,
        )
        tbm_specs, tb_params = setup_digital_tran(self.specs, dut, **tb_params)
        tbm = self.make_tbm(DigitalTranTB, tbm_specs)
        tbm.sim_params['v_incm'] = v_incm
        tbm.sim_params['v_tail_g'] = v_tail_g
        sim_results = await sim_db.async_simulate_tbm_obj(name, sim_dir, dut, tbm, tb_params, harnesses=harnesses)
        return self.process_results(sim_results)

    @staticmethod
    def process_results(sim_results: SimResults) -> Mapping[str, Any]:
        sim_data = sim_results.data
        time = sim_data['time']

        # compute average current consumption
        i_vdd = - sim_data['VDC_VDD:p'][0]
        i_avg = np.trapz(i_vdd, time) / time[-1]

        # analyze eye
        out_d = sim_data['i_outp'][0] - sim_data['i_outm'][0]
        tbm = sim_results.tbm
        t_per = tbm.sim_params['t_per']
        t_delay = tbm.sim_params['t_delay']
        eye_ana = EyeAnalysis(t_per, t_delay)

        return dict(
            eye=eye_ana.analyze_eye(time, out_d),
            i_avg=i_avg,
        )

    @staticmethod
    def plot_results(results: Mapping[str, Any]) -> None:
        sim_envs: Sequence[str] = results['sim_envs']
        v_incm_swp: np.ndarray = results['v_incm']
        v_tail_g_swp: np.ndarray = results['v_tail_g']
        fig, ax_list = plt.subplots(len(sim_envs), len(v_incm_swp) * len(v_tail_g_swp))
        if not isinstance(ax_list, np.ndarray):
            ax_list = np.array([ax_list]).reshape((1, 1))
        if len(v_incm_swp) * len(v_tail_g_swp) == 1:
            ax_list = ax_list.reshape(len(sim_envs), 1)

        for jdx, sim_env in enumerate(sim_envs):
            for kdx, v_incm in enumerate(v_incm_swp):
                for ldx, v_tail_g in enumerate(v_tail_g_swp):
                    eidx = kdx * len(v_tail_g_swp) + ldx
                    ax = ax_list[jdx, eidx]
                    eye_results: EyeResults = results['eye'][jdx][eidx]
                    eye_results.plot(ax)
                    ax.set_title(f'{sim_env}; v_incm={v_incm} V; v_tail_g={v_tail_g} V')

        plt.tight_layout()
        plt.show()
