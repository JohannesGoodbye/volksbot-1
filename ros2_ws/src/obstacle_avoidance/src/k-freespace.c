/***************************************************************************
 *                            k-freespace.c
 ***************************************************************************
 *  PROJECT         : 3D Laser scanner on Kurt Robots
 *  AUTHOR          : Hartmut Surmann, Andreas Nuechter 9/2002
 ***************************************************************************
 *  DESCRIPION
 * Implementation of the code for a simple potential field / free space
 * method for  + obstacle avoidance 
 ***************************************************************************/
 
/***************************************************************************
 * INCLUDED HEADERS
 ***************************************************************************/
#include <math.h>
#include <stdio.h>

#include "k-freespace.h"
 
 
/***************************************************************************
 * CONSTANTS & MACROS
 ***************************************************************************/  
#define INVPIDIV180          57.29578               /*   180 / pi   */   
#define EPSILON              0.000001               // vergleich von 2 doubles      
#define max(a,b) ((a)<(b)?(b):(a))  

/*
 *
 *
 */
double calc_freespace (int nr, int *distance, double *x, double *y)
{
  int i;
  double phi;
  double sinsum = 0.0, cossum = 0.0;
  double orientation_weight = 0.0;
  double alpha = 0.0;

  //testen
  
  //  printf("*****************\n");
  for (i = 0; i < nr; i++) {
    // -M_PI/2.0 fuer die verschiebung auf -90 bis 90 
    // macht in der summe +M_PI/2.0#

    //    phi = (double)i / (nr - 1) * M_PI - M_PI/2.0;
    phi = atan2(y[i], x[i]) - M_PI/2.0;

    //printf("%d %f \n", i, phi);
    // cos(phi/2) gewichtet die werte nach vorne maximal und am 
    // rand entsprechend weniger (hier 0)
    // exp term fuer grosse distance ungefaehr 0 d.h. keine einfluss

    orientation_weight = cos(phi/1.2) * 1.0/(1.0+exp(-(distance[i]-300.0)/30.0));
    // kann aber nicht oben berechnet werden wegen scalierung von obigen wert
    // printf("%d %d %f %f\n",i,distance[i], INVPIDIV180 * phi, orientation_weight);
    sinsum += sin(phi)* orientation_weight;
    cossum += cos(phi)* orientation_weight;
  }
  //  printf("*****************\n");

  if (fabs(sinsum) > EPSILON) alpha = atan2(sinsum, cossum);
  
  // printf("a %f \n",INVPIDIV180 * alpha);
  // hier noch checken ob was null wird 
  //  if (fabs(Back_sinsum) > EPSILON) alpha += atan2(Back_sinsum,Back_cossum);
  /*
  printf("alpha %f sinsum %f cossum %f \n",
	    INVPIDIV180 * alpha,
	    sinsum,
	    cossum);
  */
  return alpha;
}
