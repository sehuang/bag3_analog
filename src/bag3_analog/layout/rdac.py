"""This module combines res_ladder and rdac_decoder."""

from typing import Mapping, Any, Optional, Type

from bag.util.immutable import Param
from bag.design.module import Module
from bag.layout.template import TemplateDB, TemplateBase
from bag.layout.routing.base import TrackID, TrackManager, WDictType, SpDictType

from pybag.core import Transform, BBox
from pybag.enum import RoundMode, Direction, Orientation, PinMode, Orient2D

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
        self._num_sel = -1

    @property
    def num_sel(self) -> int:
        return self._num_sel

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
            top_layer='Top metal layer for the RDAC, should be >= 7',
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
        self._num_sel = num_sel = num_sel_col + num_sel_row
        num_in = 1 << num_sel

        xxm_layer = dec_core.top_layer
        yym_layer = xxm_layer + 1
        ym_layer = xxm_layer - 1
        xm_layer = ym_layer - 1
        vm_layer = xm_layer - 1
        vm_lp = self.grid.tech_info.get_lay_purp_list(vm_layer)[0]

        top_layer: int = self.params['top_layer']
        assert top_layer >= yym_layer, f'This generator expects top_layer={top_layer} is >= {yym_layer}.'

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
        _coord0 = min(dec0_inst.get_pin('VSS')[0].bound_box.ym, dec0_inst.get_pin('VDD')[0].bound_box.ym)
        res_inst = self.add_instance(res_master, xform=Transform(dx=start_x), commit=False)
        _coord1 = min(res_inst.get_pin('VSS')[0].bound_box.ym, res_inst.get_pin('VDD')[0].bound_box.ym)
        off_y = dec_coord0 - res_coord0 + (_coord0 - _coord1)
        off_y = -(- off_y // h_pitch) * h_pitch
        res_inst.move_by(dy=off_y)
        res_inst.commit()
        tot_h = max(dec_h, res_h + off_y)

        self.set_size_from_bound_box(top_layer, BBox(0, 0, tot_w, tot_h), round_up=True)

        # --- Routing --- #
        # export select signals as WireArrays
        _sel: BBox = dec0_inst.get_pin('sel<0>')
        w_sel_vm = self.find_track_width(vm_layer, _sel.w)
        for idx in range(num_sel):
            _sel0: BBox = dec0_inst.get_pin(f'sel<{idx}>')
            _vm_tidx0 = self.grid.coord_to_track(vm_layer, _sel0.xm)
            _sel0_warr = self.add_wires(vm_layer, _vm_tidx0, lower=0, upper=_sel0.yh, width=w_sel_vm)
            if num_dec == 2:
                self.add_pin(f'sel0<{idx}>', _sel0_warr, mode=PinMode.LOWER)

                _sel1: BBox = dec1_inst.get_pin(f'sel<{idx}>')
                _vm_tidx1 = self.grid.coord_to_track(vm_layer, _sel1.xm)
                _sel1_warr = self.add_wires(vm_layer, _vm_tidx1, lower=0, upper=_sel1.yh, width=w_sel_vm)
                self.add_pin(f'sel1<{idx}>', _sel1_warr, mode=PinMode.LOWER)
            else:  # num_dec == 1
                self.add_pin(f'sel<{idx}>', _sel0_warr, mode=PinMode.LOWER)

        # export output as WireArray
        _out0: BBox = dec0_inst.get_pin('out')
        w_out_ym = self.find_track_width(ym_layer, _out0.w)
        _ym_tidx0 = self.grid.coord_to_track(ym_layer, _out0.xm)
        _out0_warr = self.add_wires(ym_layer, _ym_tidx0, lower=_out0.yl, upper=self.bound_box.yh, width=w_out_ym)
        if num_dec == 2:
            self.add_pin('out0', _out0_warr, mode=PinMode.UPPER)

            _out1: BBox = dec1_inst.get_pin('out')
            _ym_tidx1 = self.grid.coord_to_track(ym_layer, _out1.xm)
            _out1_warr = self.add_wires(ym_layer, _ym_tidx1, lower=_out1.yl, upper=self.bound_box.yh, width=w_out_ym)
            self.add_pin('out1', _out1_warr, mode=PinMode.UPPER)
        else:  # num_dec == 1:
            self.add_pin('out', _out0_warr, mode=PinMode.UPPER)

        # res_ladder output to rdac_decoder input
        for idx in range(num_in):
            self.connect_bbox_to_track_wires(Direction.LOWER, vm_lp, dec0_inst.get_pin(f'in<{idx}>'),
                                             res_inst.get_pin(f'out<{idx}>'))
            if num_dec == 2:
                self.connect_bbox_to_track_wires(Direction.LOWER, vm_lp, dec1_inst.get_pin(f'in<{idx}>'),
                                                 res_inst.get_pin(f'out<{idx}>'))

        # --- Supplies
        # get res supplies on yym_layer
        res_vss_xm = res_inst.get_all_port_pins('VSS')
        res_vdd_xm = res_inst.get_all_port_pins('VDD')
        yym_lidx = self.grid.coord_to_track(yym_layer, res_vdd_xm[0].lower, RoundMode.GREATER)
        yym_ridx = self.grid.coord_to_track(yym_layer, res_vdd_xm[0].upper, RoundMode.LESS)
        num_yym = self._tr_manager.get_num_wires_between(yym_layer, 'sup', yym_lidx, 'sup', yym_ridx, 'sup') + 2
        num_yym -= (1 - (num_yym & 1))
        yym_locs = self._tr_manager.spread_wires(yym_layer, ['sup'] * num_yym, yym_lidx, yym_ridx, ('sup', 'sup'))
        _coords = [self.grid.track_to_coord(yym_layer, _tidx) for _tidx in yym_locs]
        vss_yym, vdd_yym = [], []

        for sup_xm, sup_yym, cl in [(res_vss_xm, vss_yym, _coords[1::2]), (res_vdd_xm, vdd_yym, _coords[::2])]:
            for warr in sup_xm:
                for warr_single in warr.warr_iter():
                    sup_yym.append(self.connect_via_stack(self._tr_manager, warr_single, yym_layer, 'sup',
                                                          coord_list_o_override=cl))
        yh = self.bound_box.yh
        vss_yym = [self.connect_wires(vss_yym, lower=0, upper=yh)[0]]
        vdd_yym = [self.connect_wires(vdd_yym, lower=0, upper=yh)[0]]

        avail_lidx = self._tr_manager.get_next_track(yym_layer, yym_lidx, 'sup', 'sup', -2)
        avail_ridx = self._tr_manager.get_next_track(yym_layer, yym_ridx, 'sup', 'sup', 2)

        # get decoder supplies on yym_layer
        w_sup_yym = self._tr_manager.get_width(yym_layer, 'sup')
        for inst in dec_list:
            if inst.bound_box.xm > res_inst.bound_box.xm:
                yym_ridx = self.grid.coord_to_track(yym_layer, inst.bound_box.xh, RoundMode.LESS)
                num_yym = self._tr_manager.get_num_wires_between(yym_layer, 'sup', avail_ridx, 'sup', yym_ridx,
                                                                 'sup') + 2
                num_yym -= (num_yym & 1)
                yym_locs = self._tr_manager.spread_wires(yym_layer, ['sup'] * num_yym, avail_ridx, yym_ridx,
                                                         ('sup', 'sup'))
                _coords = [self.grid.track_to_coord(yym_layer, _tidx) for _tidx in yym_locs]
                vss_coords, vdd_coords = _coords[::2], _coords[1::2]
            else:
                yym_lidx = self.grid.coord_to_track(yym_layer, inst.bound_box.xl, RoundMode.GREATER)
                num_yym = self._tr_manager.get_num_wires_between(yym_layer, 'sup', yym_lidx, 'sup', avail_lidx,
                                                                 'sup') + 2
                num_yym -= (num_yym & 1)
                yym_locs = self._tr_manager.spread_wires(yym_layer, ['sup'] * num_yym, yym_lidx, avail_lidx,
                                                         ('sup', 'sup'))
                _coords = [self.grid.track_to_coord(yym_layer, _tidx) for _tidx in yym_locs]
                vdd_coords, vss_coords = _coords[::2], _coords[1::2]
            _vdd = self.connect_via_stack(self._tr_manager, inst.get_pin('VDD', layer=xxm_layer), yym_layer, 'sup',
                                          coord_list_o_override=vdd_coords)
            vdd_yym.append(self.extend_wires(_vdd, lower=0, upper=yh)[0])
            _vss = self.connect_via_stack(self._tr_manager, inst.get_pin('VSS', layer=xxm_layer), yym_layer, 'sup',
                                          coord_list_o_override=vss_coords)
            vss_yym.append(self.extend_wires(_vss, lower=0, upper=yh)[0])

        if top_layer == yym_layer:
            self.add_pin('VDD', vdd_yym, connect=True)
            self.add_pin('VSS', vss_yym, connect=True)
        else:
            vss_top, vdd_top = vss_yym, vdd_yym
            xh = self.bound_box.xh
            for _layer in range(yym_layer + 1, top_layer + 1):
                _idx0 = self.grid.coord_to_track(_layer, vss_top[0].lower, RoundMode.GREATER)
                _idx1 = self.grid.coord_to_track(_layer, vss_top[0].upper, RoundMode.LESS)
                _num = self._tr_manager.get_num_wires_between(_layer, 'sup', _idx0, 'sup', _idx1, 'sup') + 2
                _num -= (_num & 1)
                _num2 = _num >> 1
                _locs = self._tr_manager.spread_wires(_layer, ['sup'] * _num, _idx0, _idx1, ('sup', 'sup'))
                _p = _locs[2] - _locs[0]
                w_sup = self._tr_manager.get_width(_layer, 'sup')
                if self.grid.get_direction(_layer) is Orient2D.y:
                    _upper = yh
                else:
                    _upper = xh
                vss_top = self.connect_to_tracks(vss_top, TrackID(_layer, _locs[0], w_sup, _num2, _p),
                                                 track_lower=0, track_upper=_upper)
                vdd_top = self.connect_to_tracks(vdd_top, TrackID(_layer, _locs[1], w_sup, _num2, _p),
                                                 track_lower=0, track_upper=_upper)
            self.add_pin('VDD', vdd_top)
            self.add_pin('VSS', vss_top)

        # set schematic parameters
        self.sch_params = dict(
            res_params=res_master.sch_params,
            dec_params=dec_master.sch_params,
            num_dec=num_dec,
        )
