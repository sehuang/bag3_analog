# SPDX-License-Identifier: Apache-2.0
# Copyright 2019 Blue Cheetah Analog Design Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import Any, Dict, List, Optional, Type, cast

from pybag.enum import RoundMode, Direction, Orientation
from pybag.core import Transform, BBox

from bag.typing import TrackType
from bag.util.immutable import Param
from bag.design.module import Module
from bag.layout.routing.base import TrackID, WireArray
from bag.layout.routing.grid import RoutingGrid
from bag.layout.template import TemplateDB

from xbase.layout.res.base import ResBasePlaceInfo, ResArrayBase

from ...schematic.res_termination import bag3_analog__res_termination


class Termination(ResArrayBase):
    """An array of resistors to be used as a termination.
    Resistors are connected in parallel in the X direction (horizontally),
    series in the Y direction (vertically).

    Use export_mid to use as a differential termination

    Assumption
    - conn layer is a x-direction layer
    - Top layer is an even layer

    TODO: make the series / parallel direction a parameter.
    """
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        ResArrayBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        return bag3_analog__res_termination

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The ResBasePlaceInfo object.',
            nx_dum='Number of dummies on each side, X direction',
            ny_dum='Number of dummies on each side, Y direction',
            export_mid="True to export mid, so this can be used differentially",
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(nx_dum=0, ny_dum=0, export_mid=True)

    def draw_layout(self) -> None:
        pinfo = cast(ResBasePlaceInfo, ResBasePlaceInfo.make_place_info(self.grid, self.params['pinfo']))
        self.draw_base(pinfo)

        assert pinfo.top_layer >= pinfo.conn_layer + 2

        self._draw_main()
        self._draw_supplies()

        self.sch_params = dict(
            w=pinfo.w_res,
            l=pinfo.l_res,
            res_type=pinfo.res_type,
            nx=pinfo.nx,
            ny=pinfo.ny,
            nx_dum=self.params['nx_dum'],
            ny_dum=self.params['ny_dum'],
            export_mid=self.params['export_mid'],
            sup_name='VDD' if pinfo.res_config['sub_type_default'] == 'ntap' else 'VSS'
        )

    def _draw_main(self):
        """Connect center array and pin connections"""
        nx = self.place_info.nx
        ny = self.place_info.ny
        nx_dum = self.params['nx_dum']
        ny_dum = self.params['ny_dum']

        if self.params['export_mid']:
            assert nx >= 2 and nx % 2 == 0, "Nx must be even to export mid"

        tr_manager = self.tr_manager
        conn_layer = self.place_info.conn_layer
        vm_layer = conn_layer + 1
        xm_layer = vm_layer + 1
        top_layer = self.place_info.top_layer
        w_sig_hm = tr_manager.get_width(conn_layer, 'sig')
        w_sig_vm = tr_manager.get_width(vm_layer, 'sig')
        w_sig_xm = tr_manager.get_width(xm_layer, 'sig')

        prim_lay_purp = self.tech_cls.tech_info.get_lay_purp_list(conn_layer - 1)[0]

        # Connect array
        conn_wire_list = []
        for yidx in range(ny_dum, ny-ny_dum):
            conn_warrs = []
            for pname in ('MINUS', 'PLUS'):
                x_warrs = []
                for xidx in range(nx_dum, nx-nx_dum):
                    pport = cast(BBox, self.get_device_port(xidx, yidx, pname))
                    tidx = self.grid.coord_to_track(conn_layer, pport.ym, RoundMode.NEAREST)
                    x_warrs.append(
                        self.connect_bbox_to_tracks(Direction.LOWER, prim_lay_purp, pport,
                                                    TrackID(conn_layer, tidx, w_sig_hm)))
                conn_warrs.append(self.connect_wires(x_warrs)[0])  # All wires should be aligned
            # The array alternates MY, so we need to flip the order
            conn_warrs = [conn_warrs[1], conn_warrs[0]] if yidx % 2 else conn_warrs
            conn_wire_list.append(conn_warrs)

        # Connect array vertically
        if len(conn_wire_list) > 1:
            for pair_idx in range(len(conn_wire_list) - 1):
                plus_warr: WireArray = conn_wire_list[pair_idx][1]
                minus_warr: WireArray = conn_wire_list[pair_idx + 1][0]

                # Add a bunch of fill wires and connect
                tidx_l = self.grid.coord_to_track(vm_layer, plus_warr.lower, RoundMode.GREATER_EQ)
                tidx_r = self.grid.coord_to_track(vm_layer, plus_warr.upper, RoundMode.LESS_EQ)
                if tidx_l == tidx_r:
                    num_wires = 1
                else:
                    # Divide by 2 to satisfy DRC rules in many processes
                    num_wires = (tr_manager.get_num_wires_between(
                        vm_layer, 'sig', tidx_l, 'sig', tidx_r, 'sig') + 1) // 2 + 1
                tidx_list = tr_manager.spread_wires(vm_layer, ['sig'] * num_wires, tidx_l, tidx_r,
                                                    sp_type=('sig', 'sig'), alignment=0)

                vm_list = [self.connect_to_tracks([plus_warr, minus_warr], TrackID(vm_layer, tidx, w_sig_vm))
                           for tidx in tidx_list]

                # Add mid wire
                if self.params['export_mid'] and pair_idx == (ny - 2 * ny_dum) // 2 - 1:
                    xm_tidx = self.grid.coord_to_track(xm_layer, vm_list[0].middle, RoundMode.NEAREST)
                    xm_warr = self.connect_to_tracks(vm_list, TrackID(xm_layer, xm_tidx, w_sig_xm))
                    while xm_warr.layer_id != top_layer:
                        xm_warr = self._connect_up_2_layers(xm_warr, 'sig')
                    self.add_pin('MID', xm_warr)

        plus_warr: WireArray = conn_wire_list[-1][1]
        minus_warr: WireArray = conn_wire_list[0][0]

        # Connect up to top_layer
        assert not top_layer % 2, "Only even Top layer currently support"
        if plus_warr.layer_id > top_layer:
            raise RuntimeError("Top layer too low")
        # Add vertical layer fill and connect to next layer
        while plus_warr.layer_id != top_layer:
            plus_warr = self._connect_up_2_layers(plus_warr, 'sig')
        while minus_warr.layer_id != top_layer:
            minus_warr = self._connect_up_2_layers(minus_warr, 'sig')

        # Add pin
        self.add_pin('PLUS', plus_warr)
        self.add_pin('MINUS', minus_warr)

    def _draw_supplies(self):
        """Draw supply wires to connect to bulk and dummies"""
        nx = self.place_info.nx
        ny = self.place_info.ny
        nx_dum = self.params['nx_dum']
        ny_dum = self.params['ny_dum']

        tr_manager = self.tr_manager
        conn_layer = self.place_info.conn_layer
        vm_layer = conn_layer + 1
        w_sup_vm = tr_manager.get_width(vm_layer, 'sup')
        top_layer = self.place_info.top_layer
        prim_lay_purp = self.tech_cls.tech_info.get_lay_purp_list(conn_layer - 1)[0]

        # Draw horizontal supply wires
        # Currently BAG3 does not have detailed support for array unit-level wire specs
        # We will assume that we can draw a horizontal wire at each row's horizontal edge
        # We also assume the bulk connection is on the same layer as the pins and can be safely connected
        # to the horizontal bars.
        blk_h = self.place_info.height
        hm_warr_list = []
        for yidx in range(ny + 1):
            hm_tidx = self.grid.coord_to_track(conn_layer, blk_h * yidx, RoundMode.NEAREST)
            hm_tid = TrackID(conn_layer, hm_tidx, tr_manager.get_width(conn_layer, 'sup'))

            # Connect dummies to conn layer
            hm_warrs = []
            name = "PLUS" if yidx % 2 else 'MINUS'
            if yidx < ny:
                # Connect down
                if yidx < ny_dum or yidx > (ny - ny_dum - 1):
                    for xidx in range(nx):
                        bbox = cast(BBox, self.get_device_port(xidx, yidx, name))
                        hm_warrs.append(self.connect_bbox_to_tracks(Direction.LOWER, prim_lay_purp, bbox, hm_tid))
                else:
                    for xidx in range(nx_dum):
                        bbox = cast(BBox, self.get_device_port(xidx, yidx, name))
                        hm_warrs.append(self.connect_bbox_to_tracks(Direction.LOWER, prim_lay_purp, bbox, hm_tid))
                    for xidx in range(nx - nx_dum, nx):
                        bbox = cast(BBox, self.get_device_port(xidx, yidx, name))
                        hm_warrs.append(self.connect_bbox_to_tracks(Direction.LOWER, prim_lay_purp, bbox, hm_tid))

            if yidx > 0:
                # Connect up
                if yidx - 1 < ny_dum or yidx - 1 > (ny - ny_dum - 1):
                    for xidx in range(nx):
                        bbox = cast(BBox, self.get_device_port(xidx, yidx - 1, name))
                        hm_warrs.append(self.connect_bbox_to_tracks(Direction.LOWER, prim_lay_purp, bbox, hm_tid))
                else:
                    for xidx in range(nx_dum):
                        bbox = cast(BBox, self.get_device_port(xidx, yidx - 1, name))
                        hm_warrs.append(self.connect_bbox_to_tracks(Direction.LOWER, prim_lay_purp, bbox, hm_tid))
                    for xidx in range(nx - nx_dum, nx):
                        bbox = cast(BBox, self.get_device_port(xidx, yidx - 1, name))
                        hm_warrs.append(self.connect_bbox_to_tracks(Direction.LOWER, prim_lay_purp, bbox, hm_tid))

            # If we have bulk, connect bulk
            if self.has_substrate_port:
                # Hack to check port layer assumption
                assert self._unit.get_port('BULK').get_single_layer() == prim_lay_purp[0]

                # get_device_port does not return multiple bound boxes. This is work around.
                def _get_transform(_xidx, _yidx):
                    w = self._info.width
                    h = self._info.height
                    orient = Orientation.R0

                    dx = w * _xidx
                    dy = h * _yidx
                    if (_xidx & 1) != 0:
                        dx += w
                        orient = orient.flip_lr()
                    if (_yidx & 1) != 0:
                        dy += h
                        orient = orient.flip_ud()

                    return Transform(dx, dy, orient)
                bulk_bbox_list = self._unit.get_port('BULK').get_pins()
                for xidx in range(nx):
                    for bbox in bulk_bbox_list:
                        if yidx < ny:
                            bulk_bbox = cast(BBox, bbox).get_transform(_get_transform(xidx, yidx))
                            hm_warrs.append(
                                self.connect_bbox_to_tracks(Direction.LOWER, prim_lay_purp, bulk_bbox, hm_tid))

                        if yidx > 0:
                            bulk_bbox = cast(BBox, bbox).get_transform(_get_transform(xidx, yidx - 1))
                            hm_warrs.append(
                                self.connect_bbox_to_tracks(Direction.LOWER, prim_lay_purp, bulk_bbox, hm_tid))

            hm_warr_list.append(self.connect_wires(hm_warrs)[0])

        # Add vm connections
        blk_w = self.place_info.width
        vm_warrs = []
        for xidx in range(nx_dum + 1):
            vm_tidx = self.grid.coord_to_track(vm_layer, blk_w * xidx, RoundMode.NEAREST)
            vm_warrs.append(self.connect_to_tracks(hm_warr_list, TrackID(vm_layer, vm_tidx, w_sup_vm)))
            vm_tidx = self.grid.coord_to_track(vm_layer, blk_w * (nx - xidx), RoundMode.NEAREST)
            vm_warrs.append(self.connect_to_tracks(hm_warr_list, TrackID(vm_layer, vm_tidx, w_sup_vm)))

        # Connect the outer most two up to top layer
        # This isn't high speed or high current, so we don't need to worry
        if hm_warr_list[0].layer_id > top_layer:
            raise RuntimeError("Top layer too low")
        while hm_warr_list[0].layer_id < top_layer:
            hm_warr_list = [self._connect_up_2_layers(warr, 'sup') for warr in [hm_warr_list[0], hm_warr_list[-1]]]
            if hm_warr_list[0].layer_id == vm_layer + 1:
                self.connect_to_track_wires(hm_warr_list, vm_warrs)

        # Add pins
        sup_name = 'VDD' if cast(ResBasePlaceInfo, self.place_info).res_config['sub_type_default'] == 'ntap' else 'VSS'
        sup_warr = self.connect_wires(hm_warr_list)
        self.add_pin(sup_name, sup_warr, connect=True)

    def _connect_up_2_layers(self, warr: WireArray, w_type: str = 'sig') -> WireArray:
        """Helper function to connect warr up 2 layers. Should work for both horizontal and vertical metals"""
        tr_manager = self.tr_manager
        # Helper notation, but not necessarily h / v
        hm_layer = warr.layer_id
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        vm_w = tr_manager.get_width(vm_layer, w_type)
        xm_w = tr_manager.get_width(xm_layer, w_type)

        tidx_l = self.grid.coord_to_track(vm_layer, warr.lower, RoundMode.GREATER_EQ)
        tidx_r = self.grid.coord_to_track(vm_layer, warr.upper, RoundMode.LESS_EQ)
        if tidx_l == tidx_r:
            num_wires = 1
        else:
            # Divide by 2 to satisfy DRC rules in many processes
            num_wires = (tr_manager.get_num_wires_between(
                vm_layer, w_type, tidx_l, w_type, tidx_r, w_type) + 1) // 2 + 1
        tidx_list = tr_manager.spread_wires(vm_layer, [w_type] * num_wires, tidx_l, tidx_r,
                                            sp_type=(w_type, w_type), alignment=0)

        vm_warrs = []
        for tidx in tidx_list:
            vm_warrs.append(self.connect_to_tracks(warr, TrackID(vm_layer, tidx, vm_w)))
        xm_tidx = translate_layers(self.grid, hm_layer, warr.track_id.base_index, xm_layer)
        return self.connect_to_tracks(vm_warrs, TrackID(xm_layer, xm_tidx, xm_w))


def translate_layers(grid: RoutingGrid, lay0: int, tidx: TrackType, lay1: int, mode=RoundMode.NEAREST):
    coord = grid.track_to_coord(lay0, tidx)
    return grid.coord_to_track(lay1, coord, mode)
