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

from typing import Dict, Any, Tuple

import pkg_resources
from pathlib import Path

from bag.design.module import Module
from bag.design.database import ModuleDB
from bag.util.immutable import Param
from bag.math import float_to_si_string


# noinspection PyPep8Naming
class bag3_analog__high_pass(Module):
    """ A single high pass filter
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'high_pass.yaml')))

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
            l='unit resistor length, in resolution units',
            w='unit resistor width, in resolution units',
            intent='resistor type.',
            nser='number of resistors in series in a branch.',
            ndum='number of dummy resistors.',
            res_in_info='input metal resistor information.',
            res_out_info='output metal resistor information.',
            sub_name='substrate name. Empty string to disable.',
            cap_val='Schematic value for analogLib cap',
            extracted='True if doing LVS or extraction. Removes analogLib caps'
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            sub_name='VDD',
            cap_val=1e-9,
            extracted=False,
        )

    def design(self, l: int, w: int, intent: str, nser: int, ndum: int,
               res_in_info: Tuple[int, int, int], res_out_info: Tuple[int, int, int], sub_name: str,
               cap_val: float, extracted: bool):
        if ndum < 0 or nser <= 0:
            raise ValueError('Illegal values of ndum or nser.')

        # handle substrate pin
        if not sub_name:
            # delete substrate pin
            self.remove_pin('BULK')
        else:
            self.rename_pin('BULK', sub_name)

        # design dummy
        if ndum == 0:
            self.delete_instance('XRDUM')
        else:
            self.instances['XRDUM'].design(w=w, l=l, intent=intent)
            if ndum > 1:
                if sub_name:
                    term_list = [dict(BULK=sub_name)]
                else:
                    term_list = None
                self.array_instance('XRDUM', ['XRDUM<%d:0>' % (ndum - 1)], term_list=term_list)
            elif sub_name:
                self.reconnect_instance_terminal('XRDUM', 'BULK', sub_name)

        # design main resistors
        inst_name = 'XR'
        in_name = 'xp'
        out_name = 'bias'
        mid_name = 'mid'
        self.instances[inst_name].design(w=w, l=l, intent=intent)
        if nser == 1:
            if sub_name:
                self.reconnect_instance_terminal(inst_name, 'BULK', sub_name)
        else:
            if nser == 2:
                pos_name = '%s,%s' % (in_name, mid_name)
                neg_name = '%s,%s' % (mid_name, out_name)
            else:
                pos_name = '%s,%s<%d:0>' % (in_name, mid_name, nser - 2)
                neg_name = '%s<%d:0>,%s' % (mid_name, nser - 2, out_name)
            if sub_name:
                term_dict = dict(PLUS=pos_name, MINUS=neg_name, BULK=sub_name)
            else:
                term_dict = dict(PLUS=pos_name, MINUS=neg_name)
            name_list = ['%s<%d:0>' % (inst_name, nser - 1)]
            term_list = [term_dict]

            self.array_instance(inst_name, name_list, term_list=term_list)

        # Design capacitors
        if extracted:
            self.remove_instance('XCAP')
        else:
            cap_val_str = float_to_si_string(cap_val)
            self.instances['XCAP'].set_param('c', cap_val_str)

        # design metal resistors
        res_lay, res_w, res_l = res_in_info
        self.instances['XRMI'].design(layer=res_lay, w=res_w, l=res_l)
        res_lay, res_w, res_l = res_out_info
        self.instances['XRMO'].design(layer=res_lay, w=res_w, l=res_l)
