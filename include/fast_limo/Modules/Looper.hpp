/*
 Copyright (c) 2024 Oriol Martínez @fetty31

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef __FASTLIMO_LOOPER_HPP__
#define __FASTLIMO_LOOPER_HPP__

#include "fast_limo/Common.hpp"
#include "fast_limo/Config.hpp"

using namespace fast_limo;

class fast_limo::Looper {

    // VARIABLES

    public:

    private:

    // FUNCTIONS

    public:
        Looper();

    private:

    // SINGLETON 

    public:
        static Looper& getInstance(){
            static Looper* loop = new Looper();
            return *loop;
        }

    private:
        // Disable copy/move functionality
        Looper(const Looper&) = delete;
        Looper& operator=(const Looper&) = delete;
        Looper(Looper&&) = delete;
        Looper& operator=(Looper&&) = delete;

};

#endif