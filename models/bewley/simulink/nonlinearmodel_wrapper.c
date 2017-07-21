

/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <math.h>
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1
#define y_width 1
/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
/* extern double func(double a); */
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Output functions
 *
 */
void nonlinearmodel_Outputs_wrapper(const real_T *u0,
			real_T *y0,
			const real_T *xC)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
/* This sample sets the output equal to the input
      y0[0] = u0[0]; 
 For complex signals use: y0[0].re = u0[0].re; 
      y0[0].im = u0[0].im;
      y1[0].re = u1[0].re;
      y1[0].im = u1[0].im;
*/

y0[0] = xC[0];
y0[1] = xC[1];
y0[2] = xC[2];
y0[3] = xC[3];
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}

/*
 * Derivatives function
 *
 */
void nonlinearmodel_Derivatives_wrapper(const real_T *u0,
			real_T *y0,
			real_T *dx,
			real_T *xC)
{
/* %%%-SFUNWIZ_wrapper_Derivatives_Changes_BEGIN --- EDIT HERE TO _END */
double m__w  = 0.2;        // mass of both wheels (kg)
double m__b  = 1.66;       // mass of body (kg)
double I__b  = .069;       // inertia of body
double I__w  = .0001764;   // inertia of both wheels
double g     = 9.81;       // gravity yo. (m/s/s)
double l     = 0.181;      // length from wheels to robot's COM (meters)
double r     = 0.042;      // radius of wheel (meters)

double Sy;
double Cy;
double denom;
double phi_dd;
double theta_dd;

Sy = sin(xC[2]);
Cy = cos(xC[2]);

denom = (-m__b*m__b*r*r*l*l*Cy*Cy+(m__b*r*r+m__w*r*r+I__w)*l*l*m__b+I__b*(m__b*r*r+m__w*r*r+I__w));
phi_dd = (1/denom)*(Sy*(xC[3]*xC[3])*l*l*l*m__b*m__b*r-Cy*Sy*g*l*l*m__b*m__b*r+I__b*Sy*(xC[3]*xC[3])*l*m__b*r+Cy*l*m__b*r*u0[0]+l*l*m__b*u0[0]+I__b*u0[0]);
theta_dd = (1/denom)*(-Cy*Sy*(xC[3]*xC[3])*l*l*m__b*m__b*r*r+(m__b*r*r+m__w*r*r+I__w)*Sy*g*l*m__b-Cy*l*m__b*r*u0[0]-(m__b*r*r+m__w*r*r+I__w)*u0[0]);

dx[0] = xC[1];
dx[1] = phi_dd;
dx[2] = xC[3];
dx[3] = theta_dd;


/*
double m = 1;
double M = 5;
double L = 2;
double g = -10;
double d = 1;
double Sy;
double Cy;
Sy = sin(xC[2]);
Cy = cos(xC[2]);
dx[0] = xC[1];
dx[1] = (1/(m*L*L*(M+m*(1-Cy*Cy))))*(-1*m*m*L*L*g*Cy*Sy + m*L*L*(m*L*xC[3]*xC[3]*Sy - d*xC[1])) + m*L*L*(1/(m*L*L*(M+m*(1-Cy*Cy))))*u0[0];
dx[2] = xC[3];
dx[3] = (1/(m*L*L*(M+m*(1-Cy*Cy))))*((m+M)*m*g*L*Sy - m*L*Cy*(m*L*xC[3]*xC[3]*Sy - d*xC[1])) - m*L*Cy*(1/(m*L*L*(M+m*(1-Cy*Cy))))*u0[0];
 */
/* %%%-SFUNWIZ_wrapper_Derivatives_Changes_END --- EDIT HERE TO _BEGIN */
}
