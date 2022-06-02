# BSD 3-Clause License
#
# Copyright (c) 2018, Regents of the University of California
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

from typing import Mapping, Any, Optional

import pkg_resources
from pathlib import Path

from bag.design.module import Module
from bag.design.database import ModuleDB
from bag.util.immutable import Param

from pybag.enum import TermType


# noinspection PyPep8Naming
class bag3_analog__diff_res(Module):
    """Module for library bag3_analog cell diff_res.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'diff_res.yaml')))

    def __init__(self, database: ModuleDB, params: Param, **kwargs: Any) -> None:
        Module.__init__(self, self.yaml_file, database, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Mapping[str, str]:
        """Returns a dictionary from parameter names to descriptions.

        Returns
        -------
        param_info : Optional[Mapping[str, str]]
            dictionary from parameter names to descriptions.
        """
        return dict(
            load_params='Parameters for load resistors',
            dum_params='Optional Parameters for dummy resistors',
            sub_type='"ntap" or "ptap"',
            diff='True to have differential resistor, false to have single resistor, true by default.',
            bias_node='Common bias terminal of differential resistors, VDD by default',
            double='True to have 2 pairs of differential resistors, False by default.',
        )

    @classmethod
    def get_default_param_values(cls) -> Mapping[str, Any]:
        return dict(dum_params=None, diff=True, bias_node='VDD', double=False)

    def design(self, load_params: Mapping[str, Any], dum_params: Optional[Mapping[str, Any]], sub_type: str,
               diff: bool, bias_node: str, double: bool) -> None:
        """To be overridden by subclasses to design this module.

        This method should fill in values for all parameters in
        self.parameters.  To design instances of this module, you can
        call their design() method or any other ways you coded.

        To modify schematic structure, call:

        rename_pin()
        delete_instance()
        replace_instance_master()
        reconnect_instance_terminal()
        restore_instance()
        array_instance()
        """
        sub_name = 'VDD' if sub_type == 'ntap' else 'VSS'
        for _sub in ('VDD', 'VSS'):
            if sub_name != _sub and bias_node != _sub:
                self.remove_pin(_sub)

        # design dummy resistors
        if dum_params:
            self.design_resistor('XRDUM', **dum_params, mid='dum', plus=sub_name, minus=sub_name, bulk=sub_name)
        else:
            self.remove_instance('XRDUM')

        # design core resistors
        if double:
            self.array_instance('XRP', ['XRP0', 'XRP1'], dy=-150)
            self.array_instance('XRM', ['XRM0', 'XRM1'], dy=-150)
            self.design_resistor('XRP0', **load_params, mid='p0', plus=bias_node, bulk=sub_name, minus='rout<0>')
            self.design_resistor('XRP1', **load_params, mid='p1', plus=bias_node, bulk=sub_name, minus='rout<1>')
            self.design_resistor('XRM0', **load_params, mid='m0', plus=bias_node, bulk=sub_name, minus='rout_b<0>')
            self.design_resistor('XRM1', **load_params, mid='m1', plus=bias_node, bulk=sub_name, minus='rout_b<1>')
            self.rename_pin('rout', 'rout<1:0>')
            self.rename_pin('rout_b', 'rout_b<1:0>')
        else:
            self.design_resistor('XRP', **load_params, mid='p', plus=bias_node, bulk=sub_name)
            if diff:
                self.design_resistor('XRM', **load_params, mid='m', plus=bias_node, bulk=sub_name)
            else:
                self.remove_instance('XRM')
                self.remove_pin('rout_b')

        if bias_node != 'VDD' and bias_node != 'VSS':
            self.add_pin(bias_node, TermType.INOUT)
