//-------------------------------------------------------------------------
//                  The Motion Strategy Library (MSL)
//-------------------------------------------------------------------------
//
// Copyright (c) 2003 University of Illinois and Steven M. LaValle
// All rights reserved.
//
// Developed by:                Motion Strategy Laboratory
//                              University of Illinois
//                              http://msl.cs.uiuc.edu/msl/
//
// Versions of the Motion Strategy Library from 1999-2001 were developed
// in the Department of Computer Science, Iowa State University.
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal with the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
// 
//     * Redistributions of source code must retain the above copyright 
//       notice, this list of conditions and the following disclaimers.
//     * Redistributions in binary form must reproduce the above copyright 
//       notice, this list of conditions and the following disclaimers in 
//       the documentation and/or other materials provided with the 
//       distribution.
//     * Neither the names of the Motion Strategy Laboratory, University
//       of Illinois, nor the names of its contributors may be used to 
//       endorse or promote products derived from this Software without 
//       specific prior written permission.
//
// The software is provided "as is", without warranty of any kind,
// express or implied, including but not limited to the warranties of
// merchantability, fitness for a particular purpose and
// noninfringement.  In no event shall the contributors or copyright
// holders be liable for any claim, damages or other liability, whether
// in an action of contract, tort or otherwise, arising from, out of or
// in connection with the software or the use of other dealings with the
// software.
//
//-------------------------------------------------------------------------



//------------------------------------------------------------------------------
// MSLVectors of real numbers
//
//------------------------------------------------------------------------------

#include <math.h>
#include <ctype.h>
#include <stdlib.h>

#include "vector.h"
#define nil 0

ostream& operator<<(ostream& O, const MSLVector& v)
{ //O << v.dim() << " ";
  
  //O << v[v.dim()-1];
  for (int i=0;i<v.dim();i++) 
  {
	  O << " " << (i+1) << ":" << v[i];
  }
  O << "\n";
  return O;
}


istream& operator>>(istream& I, MSLVector& v) 
{ int d;
  I >> d;
  MSLVector w(d);
  for (int i=0;i<d;i++) I >> w[i];
  v = w;
  return I; 
} 


void error_handler(int i, const char* s) {
  cerr << s << "\n";
  exit(i);
}


void MSLVector::check_dimensions(const MSLVector& vec) const
{ if (d!=vec.d)
   error_handler(1,"MSLVector arguments have different dimensions.");
 }


MSLVector::MSLVector() 
{ d = 0; 
  v = nil;
}


MSLVector::MSLVector(int n) 
{ 
 if (n<0) error_handler(1,"MSLVector: negative dimension."); 
 d = n; 
 v = nil;
 if (d > 0)
   { v = new double[d]; 
   for(int i=0; i<d; i++) 
     v[i] = 0.0;
  }
}


MSLVector::~MSLVector() 
{   
  if (v) {
    delete[] v; 
    v = nil;
  }
}


MSLVector::MSLVector(const MSLVector& p) 
{ d = p.d; 
  v = nil;
  if (d > 0)
    { v = new double[d];
    for(int i=0; i<d; i++) v[i] = p.v[i];
   }
}



MSLVector::MSLVector(double x, double y) 
{ v = new double[2];
  d = 2;
  v[0] = x;
  v[1] = y;
 }

MSLVector::MSLVector(double x, double y, double z) 
{ v = new double[3];
  d = 3;
  v[0] = x;
  v[1] = y;
  v[2] = z;
 }


MSLVector MSLVector::rotate90() const
{ if (d !=2)  error_handler(1,"MSLVector::rotate90: dimension must be two. ");
  return MSLVector(-v[1],v[0]);
}

MSLVector MSLVector::rotate(double fi ) const
{ if (d !=2)  error_handler(1,"MSLVector::rotate: dimension must be two. ");
  double sinfi = sin(fi);
  double cosfi = cos(fi);
  return MSLVector(v[0]*cosfi-v[1]*sinfi,v[0]*sinfi+v[1]*cosfi);
}


double  MSLVector::operator[](int i) const
{ if (i<0 || i>=d)  error_handler(1,"MSLVector: index out of range ");
  return v[i]; 
}

double& MSLVector::operator[](int i)
{ if (i<0 || i>=d)  error_handler(1,"MSLVector: index out of range ");
  return v[i]; 
}


MSLVector& MSLVector::operator+=(const MSLVector& vec)
{ check_dimensions(vec);
  int n = d;
  while (n--) v[n] += vec.v[n];
  return *this;
}

MSLVector& MSLVector::operator-=(const MSLVector& vec)
{ check_dimensions(vec);
  int n = d;
  while (n--) v[n] -= vec.v[n];
  return *this;
}

MSLVector MSLVector::operator+(const MSLVector& vec) const
{ check_dimensions(vec);
  int n = d;
  MSLVector result(n);
  while (n--) result.v[n] = v[n]+vec.v[n];
  return result;
}

MSLVector MSLVector::operator-(const MSLVector& vec) const
{ check_dimensions(vec);
  int n = d;
  MSLVector result(n);
  while (n--) result.v[n] = v[n]-vec.v[n];
  return result;
}

MSLVector MSLVector::operator-() const
{ int n = d;
  MSLVector result(n);
  while (n--) result.v[n] = -v[n];
  return result;
}


MSLVector MSLVector::operator*(double x) const
{ int n = d;
  MSLVector result(n);
  while (n--) result.v[n] = v[n] * x;
  return result;
}

MSLVector MSLVector::operator/(double x) const
{ int n = d;
  MSLVector result(n);
  while (n--) result.v[n] = v[n] / x;
  return result;
}

double MSLVector::operator*(const MSLVector& vec) const
{ check_dimensions(vec);
  double result=0;
  int n = d;
  while (n--) result = result+v[n]*vec.v[n];
  return result;
}

MSLVector& MSLVector::operator=(const MSLVector& vec)
{ 
  if (d < vec.d)
  { if (v)
      delete(v);

    d = vec.d;

    if (d > 0)
      v = new double[d];
    else
      v = 0;
   }

  for(int i=0; i< vec.dim(); i++) v[i] = vec.v[i];

  return *this;
}


bool MSLVector::operator==(const MSLVector& vec)  const
{ if (vec.d != d) return false;
  int i = 0;
  while ((i<d) && (v[i]==vec.v[i])) i++;
  return (i==d) ? true : false;
}


void MSLVector::read(istream& is) 
{ for(int i=0;i<d;i++) is  >> v[i]; }

void MSLVector::print(ostream& os) 
{ os << "(";
  for(int i=0;i<d;i++) os << v[i];
  os << " )";
}


double MSLVector::sqr_length() const { return *this * *this; }

double MSLVector::length() const { return sqrt(sqr_length()); }


double MSLVector::angle(const MSLVector& y) const
{ 
  const MSLVector& x = *this;

  double L = x.length() * y.length();

  if (L == 0)
    error_handler(1,"angle: zero argument\n");

  double cosin = (x*y)/L;

  if (cosin < -1) cosin = -1;
  if (cosin >  1) cosin =  1;

  return acos(cosin);
}


