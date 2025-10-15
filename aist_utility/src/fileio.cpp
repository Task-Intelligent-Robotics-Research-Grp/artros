// Software License Agreement (BSD License)
//
// Copyright (c) 2021, National Institute of Advanced Industrial Science and Technology (AIST)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of National Institute of Advanced Industrial
//    Science and Technology (AIST) nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Toshio Ueshiba
//
/*!
 *  \file	ply.cpp
 *  \author	Toshio Ueshiba
 *  \brief	Save depth and color images to Ordered PLY file
 */
#include <filesystem>
#include <vector>
#include <stdexcept>
#include <aist_utility/fileio.hpp>

namespace aist_utility
{
std::string
url_to_filepath(const std::string& url)
{
  // Split the input URL into tokens by the dellimiter '/'.
    std::vector<std::string>	tokens;
    size_t			pos = 0;
    for (size_t epos;
	 (epos = url.find_first_of('/', pos)) != std::string::npos;
	 pos = epos + 1)
	tokens.push_back(url.substr(pos, epos - pos));
    if (pos < url.size())
	tokens.push_back(url.substr(pos));

    if (tokens.size() < 3 || tokens[1] != "")
	throw std::runtime_error("illegal URL: " + url);

    std::filesystem::path	path;
    if (tokens[0] == "package:")
        path = ament_index_cpp::get_package_share_directory(tokens[2]);
    else if (tokens[0] == "file:")
        path = "/";
    else
        throw std::runtime_error("unknown URL scheme: " + tokens[0]);

    for (size_t n = 3; n < tokens.size(); ++n)
	path /= tokens[n];

    return path.string<char>();
}
}	// namespace aist_utility
