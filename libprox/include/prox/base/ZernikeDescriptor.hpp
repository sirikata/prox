
#ifndef _ZERNIKE_DESCRIPTOR_HPP
#define _ZERNIKE_DESCRIPTOR_HPP

#include <boost/tokenizer.hpp>
#include <prox/util/Platform.hpp>
#include <iostream>
#include <math.h>
#include <string>
#include <cassert>
#include <sstream>

namespace Prox {

class ZernikeDescriptor {
 private:
   std::vector<float> mZernikeMoments;

 public:
   ZernikeDescriptor() {

   }

  ZernikeDescriptor(uint32 length, float init_value) {
    for (uint32 i=0; i<length; i++) {
       mZernikeMoments.push_back(init_value); 
     }
  }

   ZernikeDescriptor(const ZernikeDescriptor& z) {
     for (uint32 i=0; i<z.mZernikeMoments.size(); i++) {
       mZernikeMoments.push_back(z.mZernikeMoments[i]); 
     }
   }

   ZernikeDescriptor(const std::string& zernikeStr) {
     boost::char_separator<char> sep(",: ");
     boost::tokenizer<boost::char_separator<char> > tokens(zernikeStr, sep);

     for(boost::tokenizer<boost::char_separator<char> >::iterator beg=tokens.begin();
         beg!=tokens.end();++beg)
     {
       String token = *beg;
       float moment = atof(token.c_str());
       
       mZernikeMoments.push_back(moment);
     }
   }

   static ZernikeDescriptor& null() {
     static ZernikeDescriptor empty;
     return empty;
   }

   String toString() const {     
     std::ostringstream os;
     os << "[";
     for (uint32 i = 0; i <  mZernikeMoments.size(); i++) {
       os << mZernikeMoments[i] <<" , ";       
     }
     os << "]";
     return os.str();
   }

   uint32 size() const {
     return mZernikeMoments.size();
   }

   float l2Norm() const {
     float l2norm = 0;
     for (uint32 i = 0; i <  mZernikeMoments.size(); i++) {
       l2norm += (mZernikeMoments[i] * mZernikeMoments[i]);
     }

     return sqrt(l2norm);
   }

   ZernikeDescriptor minus(const ZernikeDescriptor& zd) const{
     ZernikeDescriptor new_zd = *this;
     if (size() == 0) {
       new_zd = ZernikeDescriptor(zd.size(), 0);
     }

     for (uint32 i = 0; i < zd.size() && i < new_zd.size(); i++) {
       new_zd.mZernikeMoments[i] -= zd.mZernikeMoments[i];
     }

     return new_zd;
   }

  ZernikeDescriptor plus(const ZernikeDescriptor& zd) const {           
     ZernikeDescriptor new_zd = *this;
     if (size() == 0) {
       new_zd = ZernikeDescriptor(zd.size(), 0);
     }

     for (uint32 i = 0; i < zd.size() && i < new_zd.size(); i++) {
       new_zd.mZernikeMoments[i] += zd.mZernikeMoments[i];
     }

     return new_zd;
   }

   ZernikeDescriptor multiply(float multiplier) const{
     ZernikeDescriptor new_zd = *this;

     for (uint32 i = 0; i <  new_zd.size(); i++) {
       new_zd.mZernikeMoments[i] *= multiplier;
     }

     return new_zd;
   }

   ZernikeDescriptor divide(float divisor) const {
     assert (divisor != 0);
     
     ZernikeDescriptor new_zd = *this;

     for (uint32 i = 0; i <  new_zd.size(); i++) {
       new_zd.mZernikeMoments[i] /= divisor;
     }

     return new_zd;
   }

};

}

#endif
