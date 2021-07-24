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


# noinspection PyPep8Naming
class bag3_analog__rdac_decoder(Module):
    """Module for library bag3_analog cell rdac_decoder.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'rdac_decoder.yaml')))

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
            row_params='Parameters for rdac_decoder_row',
            col_params='Parameters for rdac_decoder_col',
        )

    def design(self, row_params: Mapping[str, Any], col_params: Mapping[str, Any]) -> None:
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
        # row decoder
        self.instances['XROW'].design(**row_params)
        num_sel_row: int = row_params['num_sel']
        sel_row_suf = f'<{num_sel_row - 1}:0>'

        # column decoder
        self.instances['XCOL'].design(**col_params)
        num_sel_col: int = col_params['num_sel']
        sel_col_suf = f'<{num_sel_col - 1}:0>'

        num_sel = num_sel_row + num_sel_col
        sel_suf = f'<{num_sel - 1}:0>'
        self.rename_pin('sel', f'sel{sel_suf}')

        num_in = 1 << num_sel
        in_suf = f'<{num_in - 1}:0>'
        self.rename_pin('in', f'in{in_suf}')

        num_rows = 1 << num_sel_row
        row_suf = f'<{num_rows - 1}:0>'

        self.reconnect_instance('XROW', [(f'in{row_suf}', f'mid{row_suf}'),
                                         (f'sel{sel_row_suf}', f'sel<{num_sel - 1}:{num_sel_col}>')])

        self.reconnect_instance('XCOL', [(f'in{in_suf}', f'in{in_suf}'), (f'out{row_suf}', f'mid{row_suf}'),
                                         (f'sel{sel_col_suf}', f'sel{sel_col_suf}')])
