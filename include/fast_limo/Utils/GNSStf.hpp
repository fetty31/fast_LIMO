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

#ifndef __FASTLIMO_GNSS_TF_HPP__
#define __FASTLIMO_GNSS_TF_HPP__

#include "fast_limo/Common.hpp"

#include <GeographicLib/LocalCartesian.hpp>

class fast_limo::GNSStf {

    public:
        GNSStf() : hasInitpose(false){
            transformer_ = new GeographicLib::LocalCartesian(GeographicLib::Geocentric::WGS84());
        }

        GNSStf(double &lat, double &lng, double &alt) : hasInitpose(true) {
            transformer_ = new GeographicLib::LocalCartesian(lat, lng, alt, GeographicLib::Geocentric::WGS84());
        }

        ~GNSStf(){
            delete transformer_;
        }

        bool hasLocalAxis(){
            return this->hasInitpose;
        }

        void setInitPose(double &lat, double &lng, double &alt){
            transformer_->Reset(lat, lng, alt);
            this->hasInitpose = true;
        }

        Eigen::Vector3d getInitPose(){
            return Eigen::Vector3d( transformer_->LatitudeOrigin(), 
                                    transformer_->LongitudeOrigin(), 
                                    transformer_->HeightOrigin() );
        }

        Eigen::Vector3d getENUpose(double &lat, double &lng, double &alt){
            static double localE, localN, localU;
            this->transformer_->Forward(lat, lng, alt, localE, localN, localU);
            return Eigen::Vector3d(localE, localN, localU);
        }

        Eigen::Vector3d getLLApose(double &E, double &N, double &U){
            static double lat, lng, alt;
            this->transformer_->Reverse(E, N, U, lat, lng, alt);
            return Eigen::Vector3d(lat, lng, alt);
        }


    private:
        GeographicLib::LocalCartesian* transformer_;
        bool hasInitpose;

};

#endif