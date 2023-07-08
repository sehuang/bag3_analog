from typing import Mapping, Any, Optional, Type, Union

from bag.util.immutable import Param
from bag.design.module import Module
from bag.layout.template import TemplateDB, TemplateBase
from bag.layout.routing.base import TrackManager, WDictType, SpDictType

from pybag.enum import RoundMode, MinLenMode, Orientation
from pybag.core import Transform, BBox

from xbase.layout.mos.top import GenericWrapper
from xbase.layout.array.top import ArrayBaseWrapper

from bag3_magnetics.layout.inductor.ind_diff_wrap import IndDiffWrap

from .gm_stage import GmStageGR, GmStage
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
            ind_v_sp='Vertical spacing between resistor and inductor',
            lay_mode='Layout mode, TOP by default.',
        )

    @classmethod
    def get_default_param_values(cls) -> Mapping[str, Any]:
        return dict(lay_mode=LayMode.TOP, ind_v_sp=0)

    def draw_layout(self) -> None:
        # make masters
        gm_params: Mapping[str, Any] = self.params['gm_params']
        gm_cls = GmStageGR if self.has_guard_ring else GmStage
        gm_master = self.new_template(GenericWrapper, params=dict(cls_name=gm_cls.get_qualified_name(),
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

        ind_v_sp: int = self.params['ind_v_sp']
        x_ind = (w_tot + ind_sh_bbox.w) // 2
        y_ind = y_res + res_bbox.h + ind_v_sp
        ind = self.add_instance(ind_sh_master, xform=Transform(dx=x_ind, dy=y_ind, mode=Orientation.MY), commit=gen_ind)

        h_tot = ind.bound_box.yh
        self.set_size_from_bound_box(ind_layer, BBox(0, 0, w_tot, h_tot), round_up=True)

        # --- Routing --- #
        # gm cell
        if gen_gm:
            for pin in ('v_inp', 'v_inm', 'v_tail_g', 'v_tail'):
                self.reexport(gm.get_port(pin))

            self.reexport(gm.get_port('VSS'), connect=True)
            if gm.has_port('VDD'):
                # TODO: routing supply
                self.reexport(gm.get_port('VDD'), connect=True)

        # res_diff
        if gen_res:
            for pin in ('VDD', 'VSS'):
                if res.has_port(pin):
                    self.reexport(res.get_port(pin), connect=True)

        # routing from gm cell to res_diff
        i_outp = gm.get_pin('i_outp')
        i_outm = gm.get_pin('i_outm')
        gm_top_layer = i_outp.layer_id

        p_in = res.get_pin('p_in')
        p_out = res.get_pin('p_out')
        m_in = res.get_pin('m_in')
        m_out = res.get_pin('m_out')

        if gm_top_layer < p_in.layer_id:
            raise NotImplementedError
        elif gm_top_layer > p_in.layer_id:
            _tidx0 = self.grid.coord_to_track(gm_top_layer, p_in.bound_box.ym, RoundMode.NEAREST)
            _tidx1 = self.grid.coord_to_track(gm_top_layer, p_out.bound_box.ym, RoundMode.NEAREST)
            if self._tr_manager.get_next_track(gm_top_layer, _tidx0, 'sig_hs', 'sig_hs', 2) > _tidx1:
                # compute new _tidx0 and _tidx1
                _, _locs = self._tr_manager.place_wires(gm_top_layer, ['sig_hs', 'sig_hs', 'sig_hs'],
                                                        center_coord=(p_in.bound_box.ym + p_out.bound_box.ym) // 2)
                _tidx0, _tidx1 = _locs[0], _locs[-1]
                alternate_o = True
            else:
                alternate_o = False
            _coord0 = self.grid.track_to_coord(gm_top_layer, _tidx0)
            _coord1 = self.grid.track_to_coord(gm_top_layer, _tidx1)
            p_0, p_1 = p_in.lower, p_in.upper
            m_0, m_1 = m_in.lower, m_in.upper
            p_in = self.connect_via_stack(self._tr_manager, p_in, gm_top_layer, 'sig_hs',
                                          coord_list_p_override=[_coord0], alignment_o=1)
            p_in = self.extend_wires(p_in, lower=p_0, upper=p_1)[0]
            m_in = self.connect_via_stack(self._tr_manager, m_in, gm_top_layer, 'sig_hs',
                                          coord_list_p_override=[_coord0], alignment_o=-1)
            m_in = self.extend_wires(m_in, lower=m_0, upper=m_1)[0]
            p_out = self.connect_via_stack(self._tr_manager, p_out, gm_top_layer, 'sig_hs',
                                           coord_list_p_override=[_coord1], alternate_o=alternate_o, alignment_o=1)
            p_out = self.extend_wires(p_out, lower=p_0, upper=p_1)[0]
            m_out = self.connect_via_stack(self._tr_manager, m_out, gm_top_layer, 'sig_hs',
                                           coord_list_p_override=[_coord1], alternate_o=alternate_o, alignment_o=-1)
            m_out = self.extend_wires(m_out, lower=m_0, upper=m_1)[0]

        # now i_outp and p_in are on the same layer
        _conn_layer = gm_top_layer + 1
        p_in = self.connect_via_stack(self._tr_manager, p_in, _conn_layer, 'sig_hs',
                                      mlm_dict={_conn_layer: MinLenMode.LOWER})
        m_in = self.connect_via_stack(self._tr_manager, m_in, _conn_layer, 'sig_hs',
                                      mlm_dict={_conn_layer: MinLenMode.LOWER})
        i_outp = self.connect_to_track_wires(i_outp, p_in)
        i_outm = self.connect_to_track_wires(i_outm, m_in)
        if gen_gm and gen_res:
            self.add_pin('i_outp', i_outp)
            self.add_pin('i_outm', i_outm)

        # routing from res_diff to inductors
        _tidx = self.grid.coord_to_track(_conn_layer + 1, p_out.bound_box.ym, RoundMode.GREATER)
        _coord = self.grid.track_to_coord(_conn_layer + 1, _tidx)
        p_out = self.connect_via_stack(self._tr_manager, p_out, _conn_layer + 1, 'sig_hs',
                                       coord_list_p_override=[_coord], mlm_dict={_conn_layer: MinLenMode.UPPER})
        m_out = self.connect_via_stack(self._tr_manager, m_out, _conn_layer + 1, 'sig_hs',
                                       coord_list_p_override=[_coord], mlm_dict={_conn_layer: MinLenMode.UPPER})
        ind_p: BBox = ind.get_pin('P1')
        ind_m: BBox = ind.get_pin('P3')
        ind_lp = self.grid.tech_info.get_lay_purp_list(ind_layer)[0]
        if _conn_layer + 1 == ind_layer:
            self.add_rect(ind_lp, BBox(ind_p.xl, p_out.bound_box.yl, ind_p.xh, ind_p.yh))
            self.add_rect(ind_lp, BBox(ind_m.xl, m_out.bound_box.yl, ind_m.xh, ind_m.yh))
        else:
            raise NotImplementedError

        # ind
        if gen_res:
            self.reexport(ind.get_port('P1'), net_name='ind_p')
            self.reexport(ind.get_port('P3'), net_name='ind_m')
        if gen_ind:
            if lay_mode is LayMode.TOP:
                for pin in ('P13_R', 'P2_R', 'P2'):
                    # TODO: routing supplies
                    self.reexport(ind.get_port(pin), net_name='VDD', connect=True)
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
