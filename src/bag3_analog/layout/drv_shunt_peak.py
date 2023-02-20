from typing import Mapping, Any, Optional, Type, Sequence, Tuple, List, Union

from bag.util.immutable import Param
from bag.util.math import HalfInt
from bag.design.module import Module
from bag.layout.core import PyLayInstance
from bag.layout.template import TemplateDB, TemplateBase
from bag.layout.routing.base import TrackID, TrackManager, WDictType, SpDictType, WireArray

from pybag.enum import PinMode, RoundMode, Direction, MinLenMode, Orientation
from pybag.core import Transform, BBox

from xbase.layout.mos.top import GenericWrapper
from xbase.layout.array.top import ArrayBaseWrapper

from bag3_magnetics.layout.inductor.ind_diff_wrap import IndDiffWrap

from .gm_stage import GmStageGR
from .res_diff import ResDiff

from ..schematic.drv_shunt_peak import bag3_analog__drv_shunt_peak, LayMode


class DrvShuntPeak(TemplateBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)
        tr_widths: WDictType = self.params['tr_widths']
        tr_spaces: SpDictType = self.params['tr_spaces']
        self._tr_manager = TrackManager(self.grid, tr_widths, tr_spaces)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        return bag3_analog__drv_shunt_peak

    @classmethod
    def get_params_info(cls) -> Mapping[str, str]:
        return dict(
            tr_widths='Track width dictionary for TrackManager',
            tr_spaces='Track spaces dictionary for TrackManager',
            gm_params='Parameters for gm cells',
            res_params='Parameters for differential resistors',
            ind_sh_params='Parameters for shunt differential inductor',
            lay_mode='Layout mode, TOP by default.',
        )

    @classmethod
    def get_default_param_values(cls) -> Mapping[str, Any]:
        return dict(lay_mode=LayMode.TOP)

    def draw_layout(self) -> None:
        # make masters
        gm_params: Mapping[str, Any] = self.params['gm_params']
        gm_master = self.new_template(GenericWrapper, params=dict(cls_name=GmStageGR.get_qualified_name(),
                                                                  params=gm_params, export_hidden=True))
        gm_bbox = gm_master.bound_box

        res_params: Mapping[str, Any] = self.params['res_params']
        res_master = self.new_template(ArrayBaseWrapper, params=dict(cls_name=ResDiff.get_qualified_name(),
                                                                     params=res_params, export_hidden=True))
        res_bbox = res_master.bound_box

        ind_sh_params: Mapping[str, Any] = self.params['ind_sh_params']
        ind_sh_master: IndDiffWrap = self.new_template(IndDiffWrap, params=ind_sh_params)
        ind_sh_bbox = ind_sh_master.actual_bbox
        ind_layer: int = ind_sh_params['lay_id']

        lay_mode: Union[str, LayMode] = self.params['lay_mode']
        if isinstance(lay_mode, str):
            lay_mode = LayMode[lay_mode]
        gen_gm = gen_res = lay_mode is LayMode.TOP or lay_mode is LayMode.EXT
        gen_ind = lay_mode is LayMode.TOP or lay_mode is LayMode.EM

        # --- Placement --- #
        w_tot = max(gm_bbox.w, res_bbox.w, ind_sh_bbox.w)

        x_gm = (w_tot - gm_bbox.w) // 2
        y_gm = gm_bbox.h
        gm = self.add_instance(gm_master, xform=Transform(dx=x_gm, dy=y_gm, mode=Orientation.MX), commit=gen_gm)

        x_res = (w_tot + res_bbox.w) // 2
        y_res = y_gm
        res = self.add_instance(res_master, xform=Transform(dx=x_res, dy=y_res, mode=Orientation.MY), commit=gen_res)

        x_ind = (w_tot - ind_sh_bbox.w) // 2
        y_ind = y_res + res_bbox.h
        ind = self.add_instance(ind_sh_master, xform=Transform(dx=x_ind, dy=y_ind), commit=gen_ind)

        h_tot = ind.bound_box.yh
        self.set_size_from_bound_box(ind_layer, BBox(0, 0, w_tot, h_tot), round_up=True)

        # --- Routing --- #
        # gm cell
        if gen_gm:
            for pin in ('v_inp', 'v_inm', 'v_tail_g', 'v_tail'):
                self.reexport(gm.get_port(pin))

            for pin in ('i_outp', 'i_outm', 'VSS', 'VDD'):
                # TODO: routing
                self.reexport(gm.get_port(pin), connect=True)

        # res_diff
        if gen_res:
            for term, net in [('p_in', 'i_outp'), ('m_in', 'i_outm'), ('p_out', 'ind_p'), ('m_out', 'ind_m')]:
                # TODO: routing
                self.reexport(res.get_port(term), net_name=net, connect=True)
            self.reexport(res.get_port('VDD'), connect=True)

        # ind
        if gen_ind:
            if lay_mode is LayMode.TOP:
                for pin in ('P13_R', 'P2_R', 'P2'):
                    # TODO: routing
                    self.reexport(ind.get_port(pin), net_name='VDD', connect=True)
                self.reexport(ind.get_port('P1'), net_name='ind_p', connect=True)
                self.reexport(ind.get_port('P3'), net_name='ind_m', connect=True)
            else:
                for pin in ('P13_R', 'P2_R', 'P2', 'P1', 'P3'):
                    self.reexport(ind.get_port(pin))

        # get schematic parameters
        self.sch_params = dict(
            gm=gm_master.sch_params,
            res=res_master.sch_params,
            ind=ind_sh_master.sch_params,
            lay_mode=lay_mode,
        )
