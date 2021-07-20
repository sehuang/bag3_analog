"""This module contains layout generator for the RDAC decoder"""

from typing import Mapping, Any, Optional, Type

from pybag.enum import MinLenMode, RoundMode, PinMode

from bag.util.immutable import Param
from bag.design.module import Module
from bag.layout.template import TemplateDB
from bag.layout.routing.base import TrackID

from xbase.layout.enum import MOSWireType
from xbase.layout.mos.base import MOSBasePlaceInfo, MOSBase

from bag3_digital.layout.stdcells.gates import InvCore, PassGateCore
from bag3_digital.layout.stdcells.and_complex import AndComplex

from ..schematic.rdac_decoder import bag3_analog__rdac_decoder


class RDACDecoder(MOSBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        return bag3_analog__rdac_decoder

    @classmethod
    def get_params_info(cls) -> Mapping[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='Dictionary of segments of sub cell components',
            num_sel='Number of select inputs',
        )

    def draw_layout(self) -> None:
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        seg_dict: Mapping[str, Any] = self.params['seg_dict']
        num_sel: int = self.params['num_sel']
        num_in = 1 << num_sel

        # make masters
        pd0_tidx = self.get_track_index(1, MOSWireType.DS, 'sig', 0)
        pg_tidx = self.get_track_index(1, MOSWireType.G, 'sig', 0)
        nd1_tidx = self.get_track_index(0, MOSWireType.DS, 'sig', -1)
        inv_params_e = dict(pinfo=pinfo, seg=seg_dict['inv'], vertical_out=False)
        inv_params_o = dict(pinfo=pinfo, seg=seg_dict['inv'], vertical_out=False,
                            sig_locs={'pout': pd0_tidx, 'nout': nd1_tidx, 'nin': pg_tidx})
        inv_master_e = self.new_template(InvCore, params=inv_params_e)
        inv_master_o = self.new_template(InvCore, params=inv_params_o)
        inv_ncols = inv_master_e.num_cols

        pg_params = dict(pinfo=pinfo, seg=seg_dict['pg'], vertical_out=False)
        pg_master = self.new_template(PassGateCore, params=pg_params)
        pg_ncols = pg_master.num_cols

        and_params = dict(pinfo=pinfo, seg_dict=seg_dict['and'], num_in=num_sel)
        and_master: AndComplex = self.new_template(AndComplex, params=and_params)
        and_ncols = and_master.num_cols

        tap_ncols = self.get_tap_ncol()
        tot_ncols = max(and_ncols + self.min_sep_col + pg_ncols,
                        num_sel * inv_ncols + (num_sel - 1) * self.min_sep_col) \
                    + 2 * (tap_ncols + self.sub_sep_col) + self.min_sep_col

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xxm_layer = ym_layer + 1

        # --- Placement --- #
        vdd_tap_list, vss_tap_list = [], []
        vdd_hm_list, vss_hm_list = [], []

        # tile 1 and above place left tap, and_complex, passgate, right tap
        w_sig_vm = self.tr_manager.get_width(vm_layer, 'sig')
        w_sig_xm = self.tr_manager.get_width(xm_layer, 'sig')
        w_sig_ym = self.tr_manager.get_width(ym_layer, 'sig')
        w_sig_xxm = self.tr_manager.get_width(xxm_layer, 'sig')
        sel_vm_locs, selb_vm_locs = [], []
        sel_hm_list = [[] for _ in range(num_sel)]
        selb_hm_list = [[] for _ in range(num_sel)]
        pg_vm_locs, pg_ym_locs = [], []
        pg_out_vm_list, pg_out_ym_list = [], []
        and_in_list = and_master.nand_in_list
        for idx in range(num_in):
            tile_idx = idx + 1
            self.add_tap(0, vdd_tap_list, vss_tap_list, tile_idx=tile_idx)
            self.add_tap(tot_ncols, vdd_tap_list, vss_tap_list, tile_idx=tile_idx, flip_lr=True)

            cur_col = tap_ncols + self.sub_sep_col + self.min_sep_col // 2
            _and = self.add_tile(and_master, tile_idx, cur_col)
            cur_col += and_ncols + self.min_sep_col
            _pg = self.add_tile(pg_master, tile_idx, cur_col)

            # supply
            vdd_hm_list.append(self.connect_wires([_and.get_pin('VDD'), _pg.get_pin('VDD')])[0])
            vss_hm_list.append(self.connect_wires([_and.get_pin('VSS'), _pg.get_pin('VSS')])[0])

            # pg enable
            self.connect_wires([_and.get_pin('out'), _pg.get_pin('en')])
            self.connect_wires([_and.get_pin('outb'), _pg.get_pin('enb')])

            # pg input and output
            pg_out = [_pg.get_pin('nd'), _pg.get_pin('pd')]
            if len(pg_vm_locs) == 0:
                _, pg_vm_locs = self.tr_manager.place_wires(vm_layer, ['sig', 'sig'],
                                                            center_coord=pg_out[0].bound_box.xm)
                _, pg_ym_locs = self.tr_manager.place_wires(ym_layer, ['sig', 'sig'],
                                                            center_coord=pg_out[0].bound_box.xm)
            _ym = (pg_out[0].bound_box.ym + pg_out[1].bound_box.ym) // 2
            _, pg_xm_locs = self.tr_manager.place_wires(xm_layer, ['sig', 'sig', 'sig'], center_coord=_ym)
            _, pg_xxm_locs = self.tr_manager.place_wires(xxm_layer, ['sig', 'sig', 'sig'], center_coord=_ym)

            pg_in_vm = self.connect_to_tracks(_pg.get_pin('s'), TrackID(vm_layer, pg_vm_locs[0], w_sig_vm),
                                              min_len_mode=MinLenMode.MIDDLE)
            pg_in_xm = self.connect_to_tracks(pg_in_vm, TrackID(xm_layer, pg_xm_locs[1], w_sig_xm),
                                              min_len_mode=MinLenMode.MIDDLE)
            pg_in_ym = self.connect_to_tracks(pg_in_xm, TrackID(ym_layer, pg_ym_locs[0], w_sig_ym),
                                              min_len_mode=MinLenMode.MIDDLE)
            pg_in_xxm = self.connect_to_tracks(pg_in_ym, TrackID(xxm_layer, pg_xxm_locs[1], w_sig_xxm),
                                               min_len_mode=MinLenMode.LOWER)
            self.add_pin(f'in<{idx}>', pg_in_xxm)

            pg_out_vm = self.connect_to_tracks(pg_out, TrackID(vm_layer, pg_vm_locs[-1], w_sig_vm))
            pg_out_xm = self.connect_to_tracks(pg_out_vm, TrackID(xm_layer, pg_xm_locs[0], w_sig_xm, 2,
                                                                  pg_xm_locs[-1] - pg_xm_locs[0]),
                                               min_len_mode=MinLenMode.MIDDLE)
            pg_out_ym = self.connect_to_tracks(pg_out_xm, TrackID(ym_layer, pg_ym_locs[-1], w_sig_ym))
            pg_out_vm_list.append(pg_out_vm)
            pg_out_ym_list.append(pg_out_ym)
            if idx == (num_in >> 1) - 1:
                out_xxm = self.connect_to_tracks(pg_out_ym, TrackID(xxm_layer, pg_xxm_locs[-1], w_sig_xxm),
                                                 min_len_mode=MinLenMode.UPPER)
                self.add_pin('out', out_xxm)

            # sel and selb
            if len(sel_vm_locs) == 0:
                for nand_idx, nand_in in enumerate(and_in_list):
                    _nand_out = _and.get_pin(f'nand_out{nand_idx}')
                    _, _vm_locs = self.tr_manager.place_wires(vm_layer, ['sig'] * (2 * nand_in + 1),
                                                              _nand_out.track_id.base_index, -1)
                    sel_vm_locs.extend(_vm_locs[0:-1:2])
                    selb_vm_locs.extend(_vm_locs[1:-1:2])
            bin_idx = f'{idx:0b}'.zfill(num_sel)
            for char_idx, bin_char in enumerate(bin_idx):
                sel_idx = num_sel - 1 - char_idx
                if bin_char == '1':
                    sel_hm_list[sel_idx].append(_and.get_pin(f'in<{sel_idx}>'))
                else:
                    selb_hm_list[sel_idx].append(_and.get_pin(f'in<{sel_idx}>'))
        # output
        self.connect_wires(pg_out_vm_list)
        self.connect_wires(pg_out_ym_list)

        self.set_mos_size(num_cols=tot_ncols, num_tiles=1 + num_in)

        # tile 0: left tap, inverters, right tap
        self.add_tap(0, vdd_tap_list, vss_tap_list, tile_idx=0)
        self.add_tap(tot_ncols, vdd_tap_list, vss_tap_list, tile_idx=0, flip_lr=True)

        cur_col = tap_ncols + self.sub_sep_col + self.min_sep_col // 2
        for idx in range(num_sel):
            _master = inv_master_o if (idx & 1) else inv_master_e
            _inv = self.add_tile(_master, 0, cur_col)
            vdd_hm_list.append(_inv.get_pin('VDD'))
            vss_hm_list.append(_inv.get_pin('VSS'))

            sel_hm_list[idx].append(_inv.get_pin('nin'))
            sel_vm = self.connect_to_tracks(sel_hm_list[idx], TrackID(vm_layer, sel_vm_locs[idx], w_sig_vm),
                                            track_lower=0, track_upper=self.bound_box.yh)
            self.add_pin(f'sel<{idx}>', sel_vm, mode=PinMode.LOWER)
            selb_hm_list[idx].extend([_inv.get_pin('pout'), _inv.get_pin('nout')])
            self.connect_to_tracks(selb_hm_list[idx], TrackID(vm_layer, selb_vm_locs[idx], w_sig_vm),
                                   track_lower=0, track_upper=self.bound_box.yh)

            cur_col += inv_ncols + self.min_sep_col

        # --- Routing --- #
        # supply
        vdd_hm = self.connect_wires(self.connect_to_track_wires(vdd_tap_list, vdd_hm_list))[0]
        vss_hm = self.connect_wires(self.connect_to_track_wires(vss_tap_list, vss_hm_list))[0]

        for _hm_layer in (xm_layer, xxm_layer):
            _vm_layer = _hm_layer - 1
            vss_vm_l = self.grid.coord_to_track(_vm_layer, vss_hm.lower, RoundMode.GREATER_EQ)
            vdd_vm_l = self.tr_manager.get_next_track(_vm_layer, vss_vm_l, 'sup', 'sup', up=1)
            vss_vm_r = self.grid.coord_to_track(_vm_layer, vss_hm.upper, RoundMode.LESS_EQ)
            vdd_vm_r = self.tr_manager.get_next_track(_vm_layer, vss_vm_r, 'sup', 'sup', up=-1)
            w_sup_vm = self.tr_manager.get_width(_vm_layer, 'sup')
            vdd_vm = self.connect_to_tracks(vdd_hm, TrackID(_vm_layer, vdd_vm_l, w_sup_vm, 2, vdd_vm_r - vdd_vm_l))
            vss_vm = self.connect_to_tracks(vss_hm, TrackID(_vm_layer, vss_vm_l, w_sup_vm, 2, vss_vm_r - vss_vm_l))

            vdd_hm_l = self.grid.coord_to_track(_hm_layer, vdd_hm[0].bound_box.ym, RoundMode.NEAREST)
            vdd_hm_l1 = self.grid.coord_to_track(_hm_layer, vdd_hm[1].bound_box.ym, RoundMode.NEAREST)
            w_sup_hm = self.tr_manager.get_width(_hm_layer, 'sup')
            vdd_hm = self.connect_to_tracks(vdd_vm, TrackID(_hm_layer, vdd_hm_l, w_sup_hm, vdd_hm.track_id.num,
                                                            vdd_hm_l1 - vdd_hm_l))
            vss_hm_l = self.grid.coord_to_track(_hm_layer, vss_hm[0].bound_box.ym, RoundMode.NEAREST)
            vss_hm_l1 = self.grid.coord_to_track(_hm_layer, vss_hm[1].bound_box.ym, RoundMode.NEAREST)
            vss_hm = self.connect_to_tracks(vss_vm, TrackID(_hm_layer, vss_hm_l, w_sup_hm, vss_hm.track_id.num,
                                                            vss_hm_l1 - vss_hm_l))
        self.add_pin('VDD', vdd_hm)
        self.add_pin('VSS', vss_hm)

        self.sch_params = dict(
            inv_params=inv_master_e.sch_params,
            and_params=and_master.sch_params,
            pg_params=pg_master.sch_params,
            num_sel=num_sel,
        )
