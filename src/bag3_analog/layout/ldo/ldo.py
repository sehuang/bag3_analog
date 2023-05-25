# BSD 3-Clause License
#
# Copyright (c) 2023, Regents of the University of California
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

# -*- coding: utf-8 -*-

"""This module contains layout generator for a ring DCO matrix."""

from typing import TYPE_CHECKING, Dict, Any, Set, Sequence, Optional, Union, Tuple, Mapping, Type, List

from pybag.enum import MinLenMode, PinMode, RoundMode, Orient2D, LogLevel
from pybag.core import Transform

from bag.util.math import HalfInt
from bag.util.immutable import Param
from bag.design.database import ModuleDB
from bag.design.module import Module
from bag.layout.template import TemplateDB, PyLayInstance, TemplateBase
from bag.layout.routing.base import TrackID, WireArray, TrackManager, WDictType, SpDictType, TrackType, BBox

from xbase.layout.enum import MOSWireType, MOSType
from xbase.layout.mos.base import MOSBasePlaceInfo, MOSBase
from xbase.layout.mos.data import MOSPorts
from xbase.layout.array.top import ArrayBaseWrapper
from xbase.layout.mos.placement.data import TilePatternElement

from bag3_analog.layout.res.termination import Termination
from bag3_analog.layout.ldo.ota import OTA
from bag3_analog.layout.ldo.power_mos import PowerPMOS, PowerNMOS

from bag3_analog.schematic.ldo import bag3_analog__ldo

import numpy as np

import math
import pdb

from xbase.layout.mos.top import GenericWrapper

class LDO(TemplateBase):
    def __init__(self, temp_db, params, **kwargs) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)
        tr_widths: WDictType = self.params['tr_widths']
        tr_spaces: SpDictType = self.params['tr_spaces']
        self._tr_manager = TrackManager(self.grid, tr_widths, tr_spaces)

    @classmethod
    def get_schematic_class(cls) -> Union[Module, None]:
        return bag3_analog__ldo

    @classmethod
    def get_params_info(cls):
        return dict(
            tr_widths='Track width dictionary.',
            tr_spaces='Track spacing dictionary.',
            ota_params='OTA parameters.',
            pwr_mos_params='Power MOS parameters.',
            res_params='Resistor parameters.',
            top_layer='top routing layer ID.'
        )

    def draw_layout(self):
        ota_params: Param = self.params['ota_params']
        pwr_mos_params: Param = self.params['pwr_mos_params']
        pwr_mos_type: str = pwr_mos_params['mos_type']
        res_params: Param = self.params['res_params']
        top_layer: int = self.params['top_layer']

        if pwr_mos_type in ['p', 'P', 'PMOS', 'pmos']:
            pwr_mos_master = self.new_template(GenericWrapper, params=dict(cls_name=PowerPMOS.get_qualified_name(),
                                                                           params=pwr_mos_params))
        elif pwr_mos_type in ['n', 'N', 'NMOS', 'nmos']:
            pwr_mos_master = self.new_template(GenericWrapper, params=dict(cls_name=PowerNMOS.get_qualified_name(),
                                                                           params=pwr_mos_params))
        else:
            raise ValueError(f'Unknown power MOS type: {pwr_mos_type}')

        ota_master = self.new_template(GenericWrapper, params=dict(cls_name=OTA.get_qualified_name(),
                                                                   params=ota_params))
        res_master = self.new_template(ArrayBaseWrapper, params=dict(cls_name=Termination.get_qualified_name(),
                                                                     params=res_params))

        tr_manager = self._tr_manager

        # add instances
        res_w = res_master.bound_box.xh - res_master.bound_box.xl
        res_h = res_master.bound_box.yh - res_master.bound_box.yl
        ota_w = ota_master.bound_box.xh - ota_master.bound_box.xl
        ota_h = ota_master.bound_box.yh - ota_master.bound_box.yl
        pwr_mos_w = pwr_mos_master.bound_box.xh - pwr_mos_master.bound_box.xl
        pwr_mos_h = pwr_mos_master.bound_box.yh - pwr_mos_master.bound_box.yl

        x_layers = range(pwr_mos_master.core.conn_layer + 1, top_layer + 1, 2)
        y_layers = range(x_layers[0] + 1, top_layer + 1, 2)
        vm_pitch, hm_pitch = self.grid.get_size_pitch(y_layers[0])

        pwr_mos = self.add_instance(pwr_mos_master, inst_name='XPWR',
                                    xform=Transform(self.align_to_pitch(ota_w + res_w, vm_pitch), 0))
        ota = self.add_instance(ota_master, inst_name='XOTA',
                                xform=Transform(self.align_to_pitch(res_w, vm_pitch),
                                                self.align_to_pitch((pwr_mos_h // 2) - (ota_h // 2), hm_pitch)))
        res = self.add_instance(res_master, inst_name='XRES',
                                xform=Transform(0, self.align_to_pitch((pwr_mos_h // 2) - (res_h // 2), hm_pitch)))


        # connect instances

        inter_vm_tidx = self.grid.coord_to_track(y_layers[0], ota.bound_box.xl, mode=RoundMode.NEAREST)
        inter_vm_tid = TrackID(y_layers[0], inter_vm_tidx, width=tr_manager.get_width(y_layers[0], 'sig'))
        self.connect_to_tracks([ota.get_pin('iref'), res.get_pin('MINUS')], inter_vm_tid)

        v_set_vm = self.connect_to_tracks(ota.get_pin('in_p'), inter_vm_tid, min_len_mode=MinLenMode.MIDDLE)
        # self.connect_via_stack(tr_manager, v_set_vm, y_layers[1], 'sig', dict([(layer, MinLenMode.MIDDLE) for layer in range(x_layers[0], y_layers[1] + 1)]))
        v_set_xm_via_tidx = self.grid.coord_to_track(x_layers[1], v_set_vm.middle, mode=RoundMode.NEAREST)
        v_set_xm_tidx = self.get_available_tracks(x_layers[1], res.get_pin('MINUS').track_id.base_index,
                                                  res.get_pin('PLUS')[-1].track_id.base_index, lower=0, upper=res.bound_box.xh,
                                                  width=tr_manager.get_width(x_layers[1], 'sig'),
                                                  sep=tr_manager.get_sep(x_layers[1], ('sig', 'sig')))
        dists = [(x, x - v_set_xm_via_tidx) for x in v_set_xm_tidx]
        dists.sort(key=lambda x: abs(x[1]))
        v_set_xm_tid = TrackID(x_layers[1], dists[0][0], width=tr_manager.get_width(x_layers[1], 'sig'))
        v_set_xm = self.connect_to_tracks(v_set_vm, v_set_xm_tid, min_len_mode=MinLenMode.MIDDLE)
        v_set_ym_tidx = self.grid.coord_to_track(y_layers[1], self.grid.track_to_coord(y_layers[0], v_set_vm.track_id.base_index), mode=RoundMode.NEAREST)
        v_set_ym_tid = TrackID(y_layers[1], v_set_ym_tidx, width=tr_manager.get_width(y_layers[1], 'sig'))
        # make a via stack here to save a ym track for connection
        v_set_ym = self.connect_to_tracks(v_set_xm, v_set_ym_tid, min_len_mode=MinLenMode.MIDDLE, track_lower=0)


        # self.connect_wires([pwr_mos.get_pin('VDD'), ota.get_pin('VDD')])
        ota_out = ota.get_pin('out')
        out_vm_tidx = self.grid.coord_to_track(y_layers[0], ota.bound_box.xh, mode=RoundMode.NEAREST)
        out_vm_tid = TrackID(y_layers[0], out_vm_tidx, width=tr_manager.get_width(y_layers[0], 'sig'))
        out_vm = self.connect_to_tracks(ota_out, out_vm_tid)
        new_tidx = tr_manager.get_next_track(x_layers[0], ota_out.track_id.base_index, 'sig', 'sig', up=-2)
        new_tid = TrackID(x_layers[0], new_tidx, width=tr_manager.get_width(x_layers[0], 'sig'))
        new_hm = self.connect_to_tracks(out_vm, new_tid)
        self.connect_to_track_wires(new_hm, self.get_vertical_warr(pwr_mos.get_all_port_pins('G'), y_layers[0]))

        if self.grid.get_direction(top_layer) == Orient2D.x:
            x_pitch, y_pitch = self.grid.get_size_pitch(top_layer)
        else:
            y_pitch, x_pitch = self.grid.get_size_pitch(top_layer)
        self.set_size_from_bound_box(top_layer, BBox(0, 0,
                                                     self.align_to_pitch(pwr_mos.bound_box.xh, x_pitch),
                                                     self.align_to_pitch(pwr_mos.bound_box.yh, y_pitch))
        )

        cur_vdd = pwr_mos.get_all_port_pins('VDD') + ota.get_all_port_pins('VDD')
        cur_vvdd = pwr_mos.get_all_port_pins('VDD_OUT')
        cur_vss = ota.get_all_port_pins('VSS')
        pwr_mos_center_vm_tidx = self.grid.coord_to_track(y_layers[0],
                                                          pwr_mos.bound_box.xl + pwr_mos_w // 2,
                                                          mode=RoundMode.NEAREST)
        pwr_mos_center_vm_tid = TrackID(y_layers[0], pwr_mos_center_vm_tidx,
                                        width=tr_manager.get_width(y_layers[0], 'sig'))
        inn_stack_start = self.connect_to_tracks(ota.get_pin('in_n'), pwr_mos_center_vm_tid,
                                                 min_len_mode=MinLenMode.MIDDLE)
        inn_stack_end = self.connect_via_stack(tr_manager, inn_stack_start, top_layer - 1, 'sig')
        # pfill_bbox = ota.bound_box + pwr_mos.bound_box
        sup_list = [cur_vdd, cur_vvdd, cur_vss]
        # do lower layers of power fill manually to distribute straps better
        pwr_mos_slice = BBox(pwr_mos.bound_box.xl, 0, pwr_mos.bound_box.xh, self.bound_box.yh)
        ota_slice = BBox(ota.bound_box.xl, 0, ota.bound_box.xh, self.bound_box.yh)
        ## hm_layer
        sup_list = [pwr_mos.get_all_port_pins('VDD'),
                    pwr_mos.get_all_port_pins('VDD_OUT'),
                    pwr_mos.get_all_port_pins('VSS')]
        # hm_pwr_mos_fill = self.do_multi_power_fill(x_layers[0], tr_manager, sup_list, bound_box=pwr_mos_slice)
        hm_pwr_mos_fill = sup_list
        sup_list = [ota.get_all_port_pins('VDD'),
                    ota.get_all_port_pins('VSS')]
        # hm_ota_fill = self.do_multi_power_fill(x_layers[0], tr_manager, sup_list, bound_box=ota_slice)
        hm_ota_fill = sup_list
        ## vm_layer
        vm_pwr_mos_fill = self.do_multi_power_fill(y_layers[0], tr_manager, hm_pwr_mos_fill, bound_box=pwr_mos.bound_box)
        vm_ota_fill = self.do_multi_power_fill(y_layers[0], tr_manager, hm_ota_fill, bound_box=ota.bound_box)
        ## xm_layer
        # FIXME: Margins are magic numbers, get real ones eventually
        xm_pwr_mos_fill = self.do_multi_power_fill(x_layers[1], tr_manager, vm_pwr_mos_fill, bound_box=pwr_mos.bound_box)
        xm_ota_res_fill = self.do_multi_power_fill(x_layers[1], tr_manager, vm_ota_fill, bound_box=ota_slice, x_margin=3 * self.grid.get_track_pitch(top_layer))
        sup_list = [xm_pwr_mos_fill[0] + xm_ota_res_fill[0] + res.get_all_port_pins('PLUS') + res.get_all_port_pins('VDD'),
                    xm_pwr_mos_fill[1],
                    xm_pwr_mos_fill[-1] + xm_ota_res_fill[-1]]
        for layer in range(y_layers[1], top_layer + 1):
            self.log(f'Performing power fill on layer {layer}', level=LogLevel.DEBUG)
            sup_list = self.do_multi_power_fill(layer, tr_manager, sup_list, bound_box=self.bound_box, x_margin=100, y_margin=100)
        # self.connect_to_track_wires(res.get_all_port_pins('VDD'), sup_list[0])
        # Find nearest VVDD track to tap to
        ref_tidx = self.grid.coord_to_track(top_layer, inn_stack_end.middle, mode=RoundMode.NEAREST)
        dists = [(x, x.track_id.base_index - ref_tidx) for x in sup_list[1]]
        dists.sort(key=lambda x: abs(x[1]))
        self.connect_to_track_wires(inn_stack_end, dists[0][0])
        self.add_pin('VDD', sup_list[0], connect=True)
        self.add_pin('VVDD', sup_list[1], connect=True)
        self.add_pin('VSS', sup_list[2], connect=True)
        self.add_pin('v_set', v_set_ym)
        # self.add_pin('v_set_ym', v_set_ym, hide=True)

        self.sch_params = dict(
            ota_params=ota_master.sch_params,
            pwr_mos_params=pwr_mos_master.sch_params,
            res_params=res_master.sch_params,
            pwr_mos_type=pwr_mos_type
        )

    def get_vertical_warr(self, box: Union[BBox, List[BBox]], layer_id: int) -> List[WireArray]:
        # export rst as WireArrays
        if not isinstance(box, list):
            box = [box]

        warr_list = []
        for box_item in box:
            _w = self.find_track_width(layer_id, box_item.w)
            _tidx = self.grid.coord_to_track(layer_id, box_item.xm)
            warr = self.add_wires(layer_id, _tidx, box_item.yl, box_item.yh, width=_w)
            warr_list.append(warr)
        return warr_list

    def align_to_pitch(self, coord: int, pitch: int) -> int:
        '''Aligns given coordinate to source/drain pitch.'''
        return coord // pitch * pitch