
#ifndef _ZERNIKE_DESCRIPTOR_HPP
#define _ZERNIKE_DESCRIPTOR_HPP

#include <boost/tokenizer.hpp>
#include <prox/util/Platform.hpp>
#include <iostream>
#include <fstream>
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

   ZernikeDescriptor(const std::vector<float>& vals) {
    mZernikeMoments = vals;
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

   float get(uint32 i) {
     assert (i < mZernikeMoments.size());

     return mZernikeMoments[i];
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

class DescriptorReader {
  std::map<String, ZernikeDescriptor> mZernikeDescriptorMap;
  std::map<String, ZernikeDescriptor> mTextureDescriptorMap;
  std::map<String, ZernikeDescriptor> mDominantColorMap;

public:

  DescriptorReader(const String& zernike_filename, const String& texture_filename, const String& dc_filename) {
    std::ifstream fp(zernike_filename.c_str());

    String line;
    while (fp) {
      std::getline(fp, line);

      if (line.length() <= 0) continue;

      size_t idx = line.find(" ");

      if (idx != String::npos) {
        String meshname = line.substr(0,idx);
        String descriptorStr = line.substr(idx+1);

        meshname = meshname.substr(meshname.find_last_of('/')+1);
        meshname = "meerkat:///tahirazim/apiupload/" + meshname + "/optimized/0/" + meshname;

        mZernikeDescriptorMap[meshname] = ZernikeDescriptor(descriptorStr);
      }
    }

    fp.close();

    std::ifstream fp2(texture_filename.c_str());
    while (fp2) {
      std::getline(fp2, line);

      if (line.length() <= 0) continue;

      size_t idx = line.find(" ");

      if (idx != String::npos) {
        String meshname = line.substr(0,idx);
        String descriptorStr = line.substr(idx+1);

        meshname = meshname.substr(meshname.find_last_of('/')+1);
        meshname = "meerkat:///tahirazim/apiupload/" + meshname + "/optimized/0/" + meshname;

        mTextureDescriptorMap[meshname] = ZernikeDescriptor(descriptorStr);
      }
    }

    fp2.close();

    std::ifstream fp3(dc_filename.c_str());
    std::getline(fp3, line);
    while (fp3) {
      String meshname=line;
      std::getline(fp3, line);  //texname
      String texname = line;
      String fullname = meshname + ":" + texname;
      std::getline(fp3, line);  //texname again
      std::getline(fp3, line);  //size info
      std::getline(fp3, line);  // sc (spatial coherence info)
      std::getline(fp3, line);  //percentage, color value, color variance table head.

      String value="abcde";
      int r,g,b;
      std::getline(fp3, value);

      int maxPercentage = 0;
      //parse next "value: " line.
      while (value.substr(0,6) == "value:") {

        bool currentIsMax = false;

        char valueline[1024];
        strncpy(valueline, value.c_str(), value.length()+1);
        const char* delim = " :|";
        char* p;
        char*save;
        int count = 0;

        //tokenize the line
#if SIRIKATA_PLATFORM == SIRIKATA_PLATFORM_WINDOWS
#define strtok_threadsafe strtok_s
#else
#define strtok_threadsafe strtok_r
#endif
        for (p=strtok_threadsafe(valueline, delim, &save); p; p = strtok_threadsafe(NULL, delim, &save) ) {
          if (count == 1) {
            int percentValue = atoi(p);
            if (percentValue > maxPercentage && percentValue > 90) {
              maxPercentage = percentValue;
              currentIsMax = true;
            }
          } //end if

          if (currentIsMax) {
            if (count == 2) r = atoi(p);
            if (count == 3) g = atoi(p);
            if (count == 4) b = atoi(p);
          } //end if

          count++;
        } //end for

        std::getline(fp3, value);
      } //end while

      line = value;

      //quantize the r,g,b's
      r = ((r/32)*32) + 16;
      g = ((g/32)*32) + 16;
      b = ((b/32)*32) + 16;

      //add to map.
      std::vector<float> rgb;
      rgb.push_back(r);
      rgb.push_back(g);
      rgb.push_back(b);

      //std::cout << fullname << " : " << r << " " << g << " " << b << "\n";
      if (maxPercentage != 0)
        mDominantColorMap[fullname] = ZernikeDescriptor(rgb);

      //std::cout << fullname << " : " << mDominantColorMap[fullname].toString() << "\n";
    }
    fp3.close();
  }

  ZernikeDescriptor& getZernikeDescriptor(const String& mesh) {
    if (mZernikeDescriptorMap.find(mesh) == mZernikeDescriptorMap.end()) {
      return ZernikeDescriptor::null();
    }

    return mZernikeDescriptorMap[mesh];
  }

  ZernikeDescriptor& getTextureDescriptor(const String& mesh) {
    if (mTextureDescriptorMap.find(mesh) == mTextureDescriptorMap.end()) {
      return ZernikeDescriptor::null();
    }

    return mTextureDescriptorMap[mesh];
  }

  ZernikeDescriptor& getDominantColorDescriptor(const String& mesh) {
    std::cout << mesh << " : getDominantCOlorDesc\n";
    if (mDominantColorMap.find(mesh) == mDominantColorMap.end()) {
      return ZernikeDescriptor::null();
    }

    return mDominantColorMap[mesh];
  }

  static DescriptorReader* getDescriptorReader() {
    static DescriptorReader* sDescriptorReader = NULL;

    if (sDescriptorReader == NULL) {
      std::cout << "Launching new DescriptorReader\n";
      sDescriptorReader = new DescriptorReader("/home/tazim/zernike_desc.txt", "/home/tazim/desc.txt", "/home/tazim/desc1.txt");
    }

    return sDescriptorReader;
  }

};


}

#endif
