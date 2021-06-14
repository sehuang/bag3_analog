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

from typing import Mapping, Any

import pkg_resources
from pathlib import Path

from bag.design.module import Module
from bag.design.database import ModuleDB
from bag.util.immutable import Param

from .high_pass_array import bag3_analog__high_pass_array


# noinspection PyPep8Naming
class bag3_analog__high_pass_clk(Module):
    """A HPF array with the inputs connected differentially to clocks
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'high_pass_clk.yaml')))

    def __init__(self, database: ModuleDB, params: Param, **kwargs: Any) -> None:
        Module.__init__(self, self.yaml_file, database, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Mapping[str, str]:
        return bag3_analog__high_pass_array.get_params_info()

    def design(self, narr: int, ndum: int, hp_params: Mapping) -> None:
        self.instances['XHPA'].design(narr=narr, ndum=ndum, hp_params=hp_params)

        sub_name = hp_params['sub_name']

        suf = '<%d:0>' % (narr - 1)
        term_list = []
        for name in ('out', 'bias'):
            new_name = name + suf
            self.rename_pin(name + '<0>', new_name)
            term_list.append((new_name, new_name))
        term_list.append((sub_name, sub_name))

        self.reconnect_instance('XHPA', term_net_iter=term_list)
        narr_2 = narr // 2
        clk_name = ''
        for idx in range(narr_2):
            clk_name = 'clkp,clkn,' + clk_name if idx % 2 else 'clkn,clkp,' + clk_name
        clk_name = clk_name[:-1]  # remove trailing comma
        self.reconnect_instance_terminal('XHPA', 'in' + suf, clk_name)

        self.rename_pin('BULK', sub_name)