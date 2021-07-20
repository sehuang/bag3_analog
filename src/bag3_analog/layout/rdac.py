"""This module combines res_ladder and rdac_ladder."""

from typing import Mapping, Any, Optional, Type

from bag.util.immutable import Param
from bag.design.module import Module
from bag.layout.template import TemplateDB, TemplateBase
from bag.layout.routing.base import TrackID, TrackManager, WDictType, SpDictType

from pybag.core import Transform, BBox
from pybag.enum import MinLenMode, RoundMode

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
        )

    def draw_layout(self) -> None:
        # make master
        res_params: Mapping[str, Any] = self.params['res_params']
        res_master = self.new_template(ArrayBaseWrapper, params=dict(cls_name=ResLadder.get_qualified_name(),
                                                                     params=res_params))
        dec_params: Mapping[str, Any] = self.params['dec_params']
        dec_master = self.new_template(GenericWrapper, params=dict(cls_name=RDACDecoder.get_qualified_name(),
                                                                   params=dec_params))
        dec_core: RDACDecoder = dec_master.core
        num_sel: int = dec_params['num_sel']
        num_in = 1 << num_sel
        num_in2 = num_in >> 1

        xxm_layer = dec_core.top_layer
        ym_layer = xxm_layer - 1

        # --- Placement --- #
        res_w, res_h = res_master.bound_box.w, res_master.bound_box.h
        dec_w, dec_h = dec_master.bound_box.w, dec_master.bound_box.h
        w_pitch, h_pitch = self.grid.get_size_pitch(xxm_layer)
        tot_w = res_w + dec_w
        tot_h = dec_h
        assert res_h < dec_h, 'These generator assumes RDACDecoder is taller than ResLadder.'

        dec_inst = self.add_instance(dec_master, xform=Transform(dx=res_w))
        dec_mid0 = dec_inst.get_pin(f'in<{num_in // 2 - 1}>')
        dec_mid1 = dec_inst.get_pin(f'in<{num_in // 2}>')
        dec_ym = (dec_mid0.bound_box.ym + dec_mid1.bound_box.ym) // 2
        off_y = dec_ym - res_h // 2
        off_y = -(- off_y // h_pitch) * h_pitch
        res_inst = self.add_instance(res_master, xform=Transform(dy=off_y))

        self.set_size_from_bound_box(xxm_layer, BBox(0, 0, tot_w, tot_h), round_up=True)

        # --- Routing --- #
        # select signals
        for idx in range(num_sel):
            self.reexport(dec_inst.get_port(f'sel<{idx}>'))

        # output
        self.reexport(dec_inst.get_port('out'))

        # res_ladder output to rdac_decoder input
        res_out0 = res_inst.get_pin('out<0>')
        _, ym_locs = self._tr_manager.place_wires(ym_layer, ['sig'] * num_in2, center_coord=res_out0.bound_box.xm)
        w_sig_ym = self._tr_manager.get_width(ym_layer, 'sig')
        for ym_idx in range(num_in2):
            ym_tid = TrackID(ym_layer, ym_locs[ym_idx], w_sig_ym)
            self.connect_to_tracks([res_inst.get_pin(f'out<{ym_idx}>'), dec_inst.get_pin(f'in<{ym_idx}>')], ym_tid,
                                   min_len_mode=MinLenMode.MIDDLE)
            self.connect_to_tracks([res_inst.get_pin(f'out<{ym_idx + num_in2}>'),
                                    dec_inst.get_pin(f'in<{ym_idx + num_in2}>')], ym_tid,
                                   min_len_mode=MinLenMode.MIDDLE)

        # supplies
        self.reexport(dec_inst.get_port('VDD'), connect=True)
        self.reexport(dec_inst.get_port('VSS'), connect=True)
        # get res supplies on xxm_layer
        w_sup_ym = self._tr_manager.get_width(ym_layer, 'sup')
        w_sup_xxm = self._tr_manager.get_width(xxm_layer, 'sup')
        sup_ym_lidx = self._tr_manager.get_next_track(ym_layer, ym_locs[0], 'sig', 'sup', up=-1)
        sup_ym_ridx = self._tr_manager.get_next_track(ym_layer, ym_locs[-1], 'sig', 'sup', up=1)
        sup_ym_tid = TrackID(ym_layer, sup_ym_lidx, w_sup_ym, 2, sup_ym_ridx - sup_ym_lidx)
        res_vss_xm = res_inst.get_all_port_pins('VSS')
        res_vdd_xm = res_inst.get_all_port_pins('VDD')
        res_vss_xxm, res_vdd_xxm = [], []
        for sup_xm, sup_xxm in [(res_vss_xm, res_vss_xxm), (res_vdd_xm, res_vdd_xxm)]:
            for warr in sup_xm:
                for warr_single in warr.warr_iter():
                    _ym = self.connect_to_tracks(warr_single, sup_ym_tid, min_len_mode=MinLenMode.MIDDLE)
                    _xxm_tidx = self.grid.coord_to_track(xxm_layer, _ym.middle, RoundMode.NEAREST)
                    sup_xxm.append(self.connect_to_tracks(_ym, TrackID(xxm_layer, _xxm_tidx, w_sup_xxm)))
        self.add_pin('VDD', res_vdd_xxm, connect=True)
        self.add_pin('VSS', res_vss_xxm, connect=True)

        # set schematic parameters
        self.sch_params = dict(
            res_params=res_master.sch_params,
            dec_params=dec_master.sch_params,
        )
