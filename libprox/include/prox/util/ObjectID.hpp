/*  libprox
 *  ObjectID.hpp
 *
 *  Copyright (c) 2009, Ewen Cheslack-Postava
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of libprox nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _PROX_OBJECT_ID_HPP_
#define _PROX_OBJECT_ID_HPP_

#include <prox/util/Platform.hpp>

namespace Prox {
namespace Reference {

class ObjectID {
    enum {
        UUID_SIZE = 16
    };
public:
    enum {
        static_size=UUID_SIZE
    };
    static const ObjectID& null() {
        static unsigned char data[UUID_SIZE]={0};
        static ObjectID retval (data,UUID_SIZE);
        return retval;
    }
    ObjectID(const void *data, int size)
    {
        if (size==UUID_SIZE) {
            memcpy(mID,data,UUID_SIZE);
        }else {
            memset(mID,0,UUID_SIZE);
            assert(false);
        }
    }

    void *begin(){
        return &mID[0];
    }
    void *end(){
        return (&mID[0]+UUID_SIZE);
    }

    const void *begin()const{
        return &mID[0];
    }
    const void *end()const {
        return (&mID[0]+UUID_SIZE);
    }

    bool operator<(const ObjectID& rhs) const {
        return memcmp(mID,rhs.mID,UUID_SIZE)<0;
    }
    bool operator==(const ObjectID& rhs) const {
        return memcmp(mID,rhs.mID,UUID_SIZE)==0;
    }
    bool operator!=(const ObjectID& rhs) const {
        return !(*this == rhs);
    }

    struct Hasher {
        size_t operator() (const ObjectID& objid) const {
            return *((const size_t*)objid.begin());
        }
    };

    struct Random {
        ObjectID operator() () const {
            unsigned char buf[UUID_SIZE];
            for(int i = 0; i < UUID_SIZE; i++)
                buf[i] = (unsigned char)(rand() % 256);
            return ObjectID(buf, UUID_SIZE);
        }
    };

    struct Null {
        const ObjectID& operator() () const {
            return ObjectID::null();
        }
    };

    static char _toHex(unsigned char c) {
        if (c < 10) return '0' + c;
        return 'a' + (c-10);
    }
    std::string toString() const {
        std::stringstream ss;
        for(int i = 0; i < UUID_SIZE; i++) {
            ss << _toHex(mID[i] & 0x0F);
            ss << _toHex(mID[i] >> 4);
        }
        return ss.str();
    }

private:
    ObjectID();

    unsigned char mID[UUID_SIZE];
}; // class ObjectID

inline std::ostream &operator<<(std::ostream &os, const ObjectID& objRef) {
    os << objRef.toString();
    return os;
}

} // namespace Reference
} // namespace Prox

#endif //_PROX_OBJECT_ID_HPP_
