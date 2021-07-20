from typing import Any, Mapping, Optional, Union, Sequence
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

from bag.simulation.cache import SimulationDB, DesignInstance, SimResults, MeasureResult
from bag.simulation.measure import MeasurementManager, MeasInfo
from bag.simulation.data import SimData
from bag.concurrent.util import GatherHelper
from bag.math.interpolate import LinearInterpolator

from bag3_testbenches.measurement.tran.digital import DigitalTranTB
from bag3_testbenches.measurement.digital.util import setup_digital_tran


class RDACMeas(MeasurementManager):
    def get_sim_info(self, sim_db: SimulationDB, dut: DesignInstance, cur_info: MeasInfo,
                     harnesses: Optional[Sequence[DesignInstance]] = None):
        raise NotImplementedError

    def initialize(self, sim_db: SimulationDB, dut: DesignInstance,
                   harnesses: Optional[Sequence[DesignInstance]] = None):
        raise NotImplementedError

    def process_output(self, cur_info: MeasInfo, sim_results: Union[SimResults, MeasureResult]):
        raise NotImplementedError

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
            ax1.plot(x_dnl, out_dnl, label=pvt)
        ax0.legend()
        ax1.legend()
        plt.tight_layout()
        plt.show()
