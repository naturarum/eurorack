// Copyright 2026 naturarum.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// See http://creativecommons.org/licenses/MIT/ for more information.
//
// -----------------------------------------------------------------------------
//
// Code that runs from ITCM: zero wait states, and no competition for the 16 kB
// instruction cache, which the whole audio path does not fit in. Beads copies
// the section at boot (CopyItcm in beads.cc).
//
// Off unless BEADS_ITCM is defined: the first build that ran from ITCM did not
// boot on the module (2026-11-07), not yet understood.
//
// ITCM is for ordinary functions. An inline function needs ITCM_SECTION with a
// name of its own: GCC puts all the inline functions of a file that share a
// section name into one section, kept or dropped as a whole by the linker with
// the first of them. Templates, with one function per instantiation, cannot
// go there.

#ifndef BEADS_DSP_ITCM_H_
#define BEADS_DSP_ITCM_H_

#if defined(BEADS_ITCM) && !defined(TEST)
#define ITCM __attribute__ ((section (".ramtext")))
#define ITCM_SECTION(name) __attribute__ ((section (".ramtext." #name)))
#else
#define ITCM
#define ITCM_SECTION(name)
#endif  // BEADS_ITCM && !TEST

#endif  // BEADS_DSP_ITCM_H_
