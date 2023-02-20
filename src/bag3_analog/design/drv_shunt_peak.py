from typing import Mapping, Any, Dict, Union, List, Type
import numpy as np
from copy import deepcopy

from bag.util.immutable import Param
from bag.io.sim_data import load_sim_file, save_sim_results
from bag.concurrent.util import GatherHelper
from bag.simulation.cache import DesignInstance
from bag.simulation.measure import MeasurementManager

from bag3_testbenches.design.optimize.base import OptDesigner, OptimizationError

from ..measurement.drv_shunt_peak import DrvShuntPeakTranMeas


class DrvShuntPeakDesigner(OptDesigner):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

    @classmethod
    def get_meas_var_list(cls):
        return ['width', 'height', 'i_avg', 'fom']

    async def async_design(self, **kwargs: Any) -> Mapping[str, Any]:
        await self.characterize_designs()
        fn_table, swp_order = self.make_models()
        opt_specs = self._dsn_specs['opt_specs']
        spec_constraints = {k: tuple(v) for k, v in opt_specs['spec_constraints'].items()}

        self.log(f'Performing eye_area optimization...')
        try:
            opt_x, opt_y, spec_vals = self.optimize('height', fn_table, swp_order, maximize=True, reduce_fn=np.max,
                                                    spec_constraints=spec_constraints)
            print('--------------------------')
            print(f'opt_x = {opt_x}')
            print(f'opt_y = {opt_y}')
            print(f'spec_vals = {spec_vals}')
            print('--------------------------')
        except OptimizationError as e:
            self.warn(f'Error occurred while running: {e}')

        return fn_table

    async def verify_design(self, dut: DesignInstance, dsn_params: Dict[str, Any],
                            sim_swp_params: Dict[str, Any]) -> Dict[str, Any]:
        dsn_name = self.get_design_name(dsn_params)

        gatherer = GatherHelper()
        gatherer.append(self.run_sim('tran', DrvShuntPeakTranMeas, dut, dsn_name, dsn_params, sim_swp_params))
        res_list = await gatherer.gather_err()
        res = self.aggregate_results(res_list)
        return res

    async def run_sim(self, meas_name: str, mm_cls: Type[MeasurementManager], dut: DesignInstance, dsn_name: str,
                      dsn_params: Dict[str, Any], sim_swp_params: Dict[str, Any]) -> Dict[str, Any]:
        sim_dir = self.get_meas_dir(dsn_name)
        out_dir = self.get_data_dir(dsn_name)

        res_fpath = out_dir / f'{meas_name}.hdf5'
        run_meas = self.check_run_meas(res_fpath)

        if not run_meas:
            prev_res = load_sim_file(str(res_fpath))
            self.reorder_data_swp(prev_res, self.sim_swp_order)
            return prev_res

        # setup measurement specs
        mm_specs = deepcopy(self._dsn_specs['meas_params'])
        mm_specs['sim_envs'] = sim_swp_params['corner']
        mm_specs['v_incm_swp'] = sim_swp_params['v_incm']
        mm_specs['v_tail_g_swp'] = sim_swp_params['v_tail_g']
        mm_specs['ind_specs']['sp_file_specs']['shunt']['file_name'] = dsn_params['sp_file']

        mm = self.make_mm(mm_cls, mm_specs)

        data = (await self._sim_db.async_simulate_mm_obj(meas_name, sim_dir / meas_name, dut, mm)).data

        res = self.postproc_tran(data, dsn_params)
        res['sweep_params'] = {k: self.sim_swp_order for k in res}
        res.update(sim_swp_params)

        save_sim_results(res, str(res_fpath))
        return res

    @staticmethod
    def postproc_tran(data: Mapping[str, Any], dsn_params: Dict[str, Any]) -> Dict[str, Any]:
        radius: int = int(dsn_params['radius'])
        height: float = data['height']
        i_avg: float = data['i_avg']
        return dict(
            width=data['width'],
            height=height,
            i_avg=i_avg,
            fom=height / (np.sqrt(i_avg) * np.sqrt(radius)),
        )

    @staticmethod
    def aggregate_results(res_list: List[Dict[str, Any]]) -> Dict[str, Any]:
        ans = {}
        for res in res_list:
            for k, v in res.items():
                if k == 'sweep_params':
                    if k not in ans:
                        ans[k] = {}
                    ans[k].update(v)
                elif k not in ans:
                    ans[k] = v
                elif isinstance(v, np.ndarray):
                    assert np.all(ans[k] == v)
                else:
                    assert ans[k] == v
        return ans

    @classmethod
    def get_dut_gen_specs(cls, is_lay: bool, base_gen_specs: Param,
                          gen_params: Mapping[str, Any]) -> Union[Param, Dict[str, Any]]:
        base_gen_specs = base_gen_specs.to_yaml()
        base_gen_specs['lay_mode'] = 'EXT'
        base_gen_specs['gm_params']['seg_dict']['tail'] = int(gen_params['seg_tail'])
        base_gen_specs['gm_params']['seg_dict']['gm'] = int(gen_params['seg_gm'])
        base_gen_specs['ind_sh_params']['radius_x'] = int(gen_params['radius'])
        base_gen_specs['ind_sh_params']['radius_y'] = int(gen_params['radius'])
        return base_gen_specs

    @classmethod
    def get_em_dut_gen_specs(cls, base_gen_specs: Param,
                             gen_params: Mapping[str, Any]) -> Union[Param, Dict[str, Any]]:
        base_gen_specs = base_gen_specs.to_yaml()
        base_gen_specs['lay_mode'] = 'EM'
        base_gen_specs['gm_params']['seg_dict']['tail'] = int(gen_params['seg_tail'])
        base_gen_specs['gm_params']['seg_dict']['gm'] = int(gen_params['seg_gm'])
        base_gen_specs['ind_sh_params']['radius_x'] = int(gen_params['radius'])
        base_gen_specs['ind_sh_params']['radius_y'] = int(gen_params['radius'])
        return base_gen_specs
