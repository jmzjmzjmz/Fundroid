#include <iostream>
#include <math.h>

using namespace std;

//#define REAL float
#define REAL double


inline static REAL sqr(REAL x) {
    return x*x;
}


int linreg(int n, const REAL x[], const REAL y[], REAL* m, REAL* b, REAL* r)
{
    REAL   sumx = 0.0;                        /* sum of x                      */
    REAL   sumx2 = 0.0;                       /* sum of x**2                   */
    REAL   sumxy = 0.0;                       /* sum of x * y                  */
    REAL   sumy = 0.0;                        /* sum of y                      */
    REAL   sumy2 = 0.0;                       /* sum of y**2                   */

   for (int i=0;i<n;i++)   
      { 
      sumx  += x[i];       
      sumx2 += sqr(x[i]);  
      sumxy += x[i] * y[i];
      sumy  += y[i];      
      sumy2 += sqr(y[i]); 
      } 

   REAL denom = (n * sumx2 - sqr(sumx));
   if (denom == 0) {
       // singular matrix. can't solve the problem.
       *m = 0;
       *b = 0;
       if (r) *r = 0;
       return 1;
   }

   *m = (n * sumxy  -  sumx * sumy) / denom;
   *b = (sumy * sumx2  -  sumx * sumxy) / denom;
   if (r!=NULL) {
      *r = (sumxy - sumx * sumy / n) /          /* compute correlation coeff     */
            sqrt((sumx2 - sqr(sumx)/n) *
            (sumy2 - sqr(sumy)/n));
   }

   return 0; 
}

int main()
{
    int n = 5;
    REAL x[6]= {3, 3, 5,  3,  3};
    REAL y[6]= {4, 8, 12, 16, 20};

    REAL m,b,r;
    int lineStatus = linreg(n,x,y,&m,&b,&r);
    cout << "LineStatus: " <<  lineStatus << endl;

    if(lineStatus == 1)
    {
    	cout << "Perfectly Parallel" << endl;
    }
    else
    {
    	REAL rSqured = r*r;

    	cout << "Slope: " << m << " Intercept: " << b << " R^2:" << rSqured << endl;
    }

    return 0;
}