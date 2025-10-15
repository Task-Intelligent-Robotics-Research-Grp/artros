/*
 *  \file	fileio_test.cpp
 */
#include <iostream>
#include <aist_utility/fileio.hpp>

int
main(int argc, char* argv[])
{
    std::cerr << "URL>> ";
    for (std::string url; std::cin >> url; )
    {
	const auto	path = aist_utility::filepath_from_url(url);
	std::cout << "path = " << path << std::endl;
	std::cerr << "URL>> ";
    }

    return 0;
}
