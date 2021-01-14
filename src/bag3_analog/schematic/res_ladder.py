# SPDX-License-Identifier: Apache-2.0
# Copyright 2020 Blue Cheetah Analog Design Inc.
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

# -*- coding: utf-8 -*-

from typing import Dict, Any

import pkg_resources
from pathlib import Path

from bag.design.module import Module
from bag.design.database import ModuleDB
from bag.util.immutable import Param


# noinspection PyPep8Naming
class bag3_analog__res_ladder(Module):
    """Module for library bag3_analog cell res_ladder.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'res_ladder.yaml')))

    def __init__(self, database: ModuleDB, params: Param, **kwargs: Any) -> None:
        Module.__init__(self, self.yaml_file, database, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        """Returns a dictionary from parameter names to descriptions.

        Returns
        -------
        param_info : Optional[Dict[str, str]]
            dictionary from parameter names to descriptions.
        """
        return dict(
            w="Resistor width",
            l="Resistor length",
            res_type="Resistor flavor",
            nx="Number of units in the X direction",
            ny="Number of units in the Y direction",
            nx_dum="Number of dummies on each side, X direction",
            ny_dum="Number of dummies on each side, Y direction",
            top_vdd='True to make the top connection VDD',
            bot_vss='True to make the bottom connection VSS',
            sup_name="Supply name to connect to bulk or float"
        )

    def design(self, w: int, l: int, res_type: str, nx: int, ny, nx_dum: int, ny_dum: int,
               top_vdd: bool, bot_vss: bool, sup_name: str) -> None:

        nx_core = nx - nx_dum * 2
        ny_core = ny - ny_dum * 2
        ncore = nx_core * ny_core
        ndum = nx * ny - ncore

        # Design unit cell
        self.instances['XRES'].design(w=w, l=l, intent=res_type)

        # set up core and dummies
        self.array_instance('XRES',
                            inst_term_list=[(f'XRESC', []), (f'XRES_DUM', [])], dx=0, dy=-200)

        # Array the core
        inst_term_list = [(f'XRES{idx}',
                           [('PLUS', 'VDD' if idx == ncore - 1 else f'out<{idx + 1}>'),
                            ('MINUS', f'out<{idx}>'), ('BULK', sup_name)])
                          for idx in range(ncore)]
        self.array_instance('XRESC', inst_term_list=inst_term_list, dx=100)
        # TODO: metal resistor for VSS - out<0>

        # Array the dummies
        inst_term_list = [(f'XRES_DUM{idx}', [('PLUS', sup_name), ('MINUS', sup_name), ('BULK', sup_name)]) \
                for idx in range(ndum)]
        self.array_instance('XRES_DUM', inst_term_list=inst_term_list, dx=100)

        # Rename out
        self.rename_pin('out', f'out<{ncore - 1}:0>')

        if top_vdd:
            self.rename_pin('top', 'VDD')
        if bot_vss:
            self.rename_pin('bottom', 'VSS')

        # Change bulk pin name
        if (sup_name == 'VDD' and not top_vdd) or (sup_name == 'VSS' and not bot_vss):
            self.rename_pin('BULK', sup_name)
        else:
            self.remove_pin('BULK')
