# SPDX-License-Identifier: BSD-3-Clause AND Apache-2.0
# Copyright 2018 Regents of the University of California
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

"""Some shared layout utility functions"""

from bag.typing import TrackType
from bag.layout.routing.grid import RoutingGrid
from bag.layout.template import TemplateBase
from bag.util.search import BinaryIterator

from pybag.enum import RoundMode


def translate_layers(grid: RoutingGrid, lay0: int, tidx: TrackType, lay1: int, mode=RoundMode.NEAREST):
    coord = grid.track_to_coord(lay0, tidx)
    return grid.coord_to_track(lay1, coord, mode)


def find_track_width(cls: TemplateBase, layer_id: int, width: int) -> int:
    bin_iter = BinaryIterator(low=1)
    while bin_iter.has_next():
        cur = bin_iter.get_next()
        cur_width = cls.grid.get_wire_total_width(layer_id, cur)
        if cur_width == width:
            return cur
        if cur_width < width:
            bin_iter.up()
        else:  # cur_width > width
            bin_iter.down()
    raise ValueError(f'Cannot find track width for width={width} on layer={layer_id}.')
