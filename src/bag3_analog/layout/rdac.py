"""This module combines res_ladder and rdac_decoder."""

from typing import Mapping, Any, Optional, Type

from bag.util.immutable import Param
from bag.design.module import Module
from bag.layout.template import TemplateDB, TemplateBase
from bag.layout.routing.base import TrackID, TrackManager, WDictType, SpDictType

from pybag.core import Transform, BBox
from pybag.enum import MinLenMode, RoundMode, Direction, Orientation

from xbase.layout.mos.top import GenericWrapper
from xbase.layout.array.top import ArrayBaseWrapper

from .res.ladder import ResLadder
from .rdac_decoder import RDACDecoder
from ..schematic.rdac import bag3_analog__rdac


class RDAC(TemplateBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)
        tr_widths: WDictType = self.params['tr_widths']
        tr_spaces: SpDictType = self.params['tr_spaces']
        self._tr_manager = TrackManager(self.grid, tr_widths, tr_spaces)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        return bag3_analog__rdac

    @classmethod
    def get_params_info(cls) -> Mapping[str, str]:
        return dict(
            tr_widths='Track widths specifications for track manager',
            tr_spaces='Track spaces specifications for track manager',
            res_params='Parameters for res_ladder',
            dec_params='Parameters for rdac_decoder',
            num_dec='Number of decoders for one res_ladder',
        )

    @classmethod
    def get_default_param_values(cls) -> Mapping[str, Any]:
        return dict(num_dec=1)

    def draw_layout(self) -> None:
        # make master
        res_params: Mapping[str, Any] = self.params['res_params']
        res_master = self.new_template(ArrayBaseWrapper, params=dict(cls_name=ResLadder.get_qualified_name(),
                                                                     params=res_params))
        res_core: ResLadder = res_master.core

        num_dec: int = self.params['num_dec']
        dec_params: Mapping[str, Any] = self.params['dec_params']
        dec_master = self.new_template(GenericWrapper, params=dict(cls_name=RDACDecoder.get_qualified_name(),
                                                                   params=dec_params))
        dec_core: RDACDecoder = dec_master.core
        num_sel_row: int = dec_params['num_sel_row']
        num_sel_col: int = dec_params['num_sel_col']
        num_sel = num_sel_col + num_sel_row
        num_in = 1 << num_sel

        xxm_layer = dec_core.top_layer
        ym_layer = xxm_layer - 1
        xm_layer = ym_layer - 1
        vm_layer = xm_layer - 1
        vm_lp = self.grid.tech_info.get_lay_purp_list(vm_layer)[0]

        # --- Placement --- #
        res_w, res_h = res_master.bound_box.w, res_master.bound_box.h
        res_coord0 = res_core.core_coord0
        dec_w, dec_h = dec_master.bound_box.w, dec_master.bound_box.h
        dec_coord0 = dec_core.pg_coord0
        w_pitch, h_pitch = self.grid.get_size_pitch(xxm_layer)
        tot_w = res_w + num_dec * dec_w
        assert res_coord0 < dec_coord0, 'These generator assumes RDACDecoder passgates start higher than ' \
                                        'ResLadder core.'

        if num_dec == 2:
            dec1_inst = self.add_instance(dec_master, xform=Transform(dx=dec_w, mode=Orientation.MY))
            start_x = dec_w
            dec_list = [dec1_inst]
        elif num_dec == 1:
            dec1_inst = None
            start_x = 0
            dec_list = []
        else:
            raise ValueError(f'num_dec={num_dec} is not supported yet. Use 1 or 2.')

        dec0_inst = self.add_instance(dec_master, xform=Transform(dx=start_x + res_w))
        dec_list.append(dec0_inst)
        off_y = dec_coord0 - res_coord0 - h_pitch  # TODO: hack to make resistor array align with passgate array
        res_inst = self.add_instance(res_master, xform=Transform(dx=start_x, dy=off_y))
        tot_h = max(dec_h, res_h + off_y)

        self.set_size_from_bound_box(xxm_layer, BBox(0, 0, tot_w, tot_h), round_up=True)

        # --- Routing --- #
        # select signals
        for idx in range(num_sel):
            if num_dec == 2:
                self.reexport(dec0_inst.get_port(f'sel<{idx}>'), net_name=f'sel0<{idx}>')
                self.reexport(dec1_inst.get_port(f'sel<{idx}>'), net_name=f'sel1<{idx}>')
            else:  # num_dec == 1
                self.reexport(dec0_inst.get_port(f'sel<{idx}>'))

        # output
        if num_dec == 2:
            self.reexport(dec0_inst.get_port('out'), net_name='out0')
            self.reexport(dec1_inst.get_port('out'), net_name='out1')
        else:  # num_dec == 1:
            self.reexport(dec0_inst.get_port('out'))

        # res_ladder output to rdac_decoder input
        for idx in range(num_in):
            self.connect_bbox_to_track_wires(Direction.LOWER, vm_lp, dec0_inst.get_pin(f'in<{idx}>'),
                                             res_inst.get_pin(f'out<{idx}>'))
            if num_dec == 2:
                self.connect_bbox_to_track_wires(Direction.LOWER, vm_lp, dec1_inst.get_pin(f'in<{idx}>'),
                                                 res_inst.get_pin(f'out<{idx}>'))

        # supplies
        for _dec_inst in dec_list:
            self.reexport(_dec_inst.get_port('VDD'), connect=True)
            self.reexport(_dec_inst.get_port('VSS'), connect=True)
        # get res supplies on xxm_layer
        res_vss_xm = res_inst.get_all_port_pins('VSS')
        res_vdd_xm = res_inst.get_all_port_pins('VDD')
        vdd_ym_lidx = self.grid.coord_to_track(ym_layer, res_vdd_xm[0].lower, RoundMode.GREATER_EQ)
        vss_ym_lidx = self._tr_manager.get_next_track(ym_layer, vdd_ym_lidx, 'sup', 'sup', up=1)
        vdd_ym_ridx = self.grid.coord_to_track(ym_layer, res_vdd_xm[0].upper, RoundMode.LESS_EQ)
        vss_ym_ridx = self._tr_manager.get_next_track(ym_layer, vdd_ym_ridx, 'sup', 'sup', up=-1)
        w_sup_ym = self._tr_manager.get_width(ym_layer, 'sup')
        w_sup_xxm = self._tr_manager.get_width(xxm_layer, 'sup')
        vdd_ym_tid = TrackID(ym_layer, vdd_ym_lidx, w_sup_ym, 2, vdd_ym_ridx - vdd_ym_lidx)
        vss_ym_tid = TrackID(ym_layer, vss_ym_lidx, w_sup_ym, 2, vss_ym_ridx - vss_ym_lidx)
        res_vss_xxm, res_vdd_xxm = [], []
        for sup_xm, sup_xxm, tid in [(res_vss_xm, res_vss_xxm, vss_ym_tid), (res_vdd_xm, res_vdd_xxm, vdd_ym_tid)]:
            for warr in sup_xm:
                for warr_single in warr.warr_iter():
                    _ym = self.connect_to_tracks(warr_single, tid, min_len_mode=MinLenMode.MIDDLE)
                    _xxm_tidx = self.grid.coord_to_track(xxm_layer, _ym.middle, RoundMode.NEAREST)
                    sup_xxm.append(self.connect_to_tracks(_ym, TrackID(xxm_layer, _xxm_tidx, w_sup_xxm)))
        self.add_pin('VDD', res_vdd_xxm, connect=True)
        self.add_pin('VSS', res_vss_xxm, connect=True)

        # set schematic parameters
        self.sch_params = dict(
            res_params=res_master.sch_params,
            dec_params=dec_master.sch_params,
            num_dec=num_dec,
        )
