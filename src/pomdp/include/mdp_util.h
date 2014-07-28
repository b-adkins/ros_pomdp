/**
 * @file
 *
 * Utility methods extending the functionality of A. Cassandra's libmdp.
 *
 * @date Jul 9, 2013
 * @author Bea Adkins
 */

#ifndef MDP_UTIL_H
#define MDP_UTIL_H

#include <stdio.h>
#include <stdlib.h>

#include <mdp/mdp.h>

extern void copyMatrix(const Matrix a, Matrix* b);
extern int getMatrixRow(Matrix matrix, int row, int len, double* val);
extern void fprintMatrixRow(FILE* stream, int len, double* val);
extern void fprintMatrix(FILE* stream, Matrix matrix, int len);

#endif /* MDP_UTIL_H */
