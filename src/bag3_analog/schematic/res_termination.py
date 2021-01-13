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

from pybag.enum import TermType

from bag.design.module import Module
from bag.design.database import ModuleDB
from bag.util.immutable import Param


# noinspection PyPep8Naming
class bag3_analog__res_termination(Module):
    """Module for library bag3_analog cell res_termination.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'res_termination.yaml')))

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
            export_mid="True to export mid, so this can be used differentially",
            sup_name="Supply name to connect to bulk or float"
        )

    def design(self, w: int, l: int, res_type: str, nx: int, ny, nx_dum: int, ny_dum: int,
               export_mid: bool, sup_name: str) -> None:
        # Design unit cell
        self.instances['XRES'].design(w=w, l=l, intent=res_type)

        # Array in the y direction
        self.array_instance('XRES', inst_term_list=[(f'XRES{y}', []) for y in range(ny)], dx=0, dy=-200)

        # Set appropriate connection in the x direction
        for yidx in range(ny):
            inst_term_list = []
            term = f'XRES{yidx}'
            for xidx in range(nx):
                if yidx < ny_dum or yidx > ny - ny_dum - 1 or xidx < nx_dum or xidx > nx - nx_dum - 1:
                    plus = sup_name
                    minus = sup_name
                elif yidx == ny_dum + (ny - 2 * ny_dum) // 2 - 1 and export_mid:
                    # TODO: there must be a better way to describe this
                    plus = 'PLUS' if yidx == ny_dum else f'm{yidx - 1}'
                    minus = 'MID'
                elif yidx == ny_dum + (ny - 2 * ny_dum) // 2 and export_mid:
                    plus = 'MID'
                    minus = 'MINUS' if yidx == ny - ny_dum - 1 else f'm{yidx}'
                else:
                    plus = 'PLUS' if yidx == ny_dum else f'm{yidx - 1}'
                    minus = 'MINUS' if yidx == ny - ny_dum - 1 else f'm{yidx}'
                inst_term_list.append((f'{term}_{xidx}', [('PLUS', plus), ('MINUS', minus), ('BULK', sup_name)]))
            self.array_instance(term, inst_term_list=inst_term_list, dx=200)

        # Change bulk pin name
        self.rename_pin('BULK', sup_name)

        if export_mid:
            self.add_pin('MID', TermType.inout)
