#include <string.h>

#include "mdp_util.h"

/**
 * Hard copies matrix a to matrix b, allocating the necessary memory.
 */
void copyMatrix(const Matrix a, Matrix* b)
{
  *b = newMatrix(a->num_rows, a->num_non_zero);
  Matrix _b = *b;

  memcpy(_b->mat_val, a->mat_val, _b->num_non_zero * sizeof(double));
  memcpy(_b->col, a->col, _b->num_non_zero * sizeof(int));

  memcpy(_b->row_start, a->row_start, _b->num_rows * sizeof(int));
  memcpy(_b->row_length, a->row_length, _b->num_rows * sizeof(int));
}

/**
 * Returns a non-sparse row of the matrix.
 *
 * @param matrix
 * @param row Row index
 * @param len Row length (number of columns)
 * @param val Buffer for row contents (non-spares)
 * @return 1 for success. 0 for failure.
 */
int getMatrixRow(Matrix matrix, int row, int len, double* val)
{
  if(row < 0 || row >= matrix->num_rows)
    return 0;

  int j;
  for(j = 0; j < len; j++)
  {
    val[j] = 0;
  }

  for(j = matrix->row_start[row];
      j < matrix->row_start[row] + matrix->row_length[row];
      j++ )
  {

    if(matrix->col[j] >= len)
    {
      return 0;
    }
    val[matrix->col[j]] = matrix->mat_val[j];
  }

  return 1;
}

/**
 * Prints a row of a matrix.
 *
 * @param stream File/stream to print to.
 * @param val Non-spare row values.
 * @param len Row length of matrix.
 */
void fprintMatrixRow(FILE* stream, int len, double* val)
{
  int j;
  for(j = 0; j < len; j++)
  {
    if(val[j] == 0)
      fprintf(stream, "%4i ", 0);
    else
      fprintf(stream, "%.2f ", val[j]);
  }
  fprintf(stream, "\n");
}

/**
 * Prints a matrix.
 *
 * @param stream File/stream to print to.
 * @param matrix Matrix to print.
 * @param len Row length of matrix.
 */
void fprintMatrix(FILE* stream, Matrix matrix, int len)
{
  double* val = malloc(sizeof(double) * matrix->num_rows);

  int i;
  for(i = 0; i < matrix->num_rows; i++)
  {

    getMatrixRow(matrix, i, len, val);
    fprintMatrixRow(stream, len, val);
  }

  free(val);
}
