/********************************************************************
* Description: interp_execute.cc
*
*   Derived from a work by Thomas Kramer
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
* Last change:
* $Revision$
* $Author$
* $Date$
********************************************************************/
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "rs274ngc.hh"
#include "rs274ngc_return.hh"
#include "rs274ngc_errors.cc"
#include "interp_internal.hh"

/****************************************************************************/

/*! execute binary

Returned value: int
   If execute_binary1 or execute_binary2 returns an error code, this
   returns that code.
   Otherwise, it returns RS274NGC_OK.

Side effects: The value of left is set to the result of applying
  the operation to left and right.

Called by: read_real_expression

This just calls either execute_binary1 or execute_binary2.

*/

int Interp::execute_binary(double *left, int operation, double *right)
{
  static char name[] = "execute_binary";
  int status;

  if (operation < AND2)
    CHP(execute_binary1(left, operation, right));
  else
    CHP(execute_binary2(left, operation, right));
  return RS274NGC_OK;
}

/****************************************************************************/

/*! execute_binary1

Returned Value: int
   If any of the following errors occur, this returns the error shown.
   Otherwise, it returns RS274NGC_OK.
   1. operation is unknown: NCE_BUG_UNKNOWN_OPERATION
   2. An attempt is made to divide by zero: NCE_ATTEMPT_TO_DIVIDE_BY_ZERO
   3. An attempt is made to raise a negative number to a non-integer power:
      NCE_ATTEMPT_TO_RAISE_NEGATIVE_TO_NON_INTEGER_POWER

Side effects:
   The result from performing the operation is put into what left points at.

Called by: read_real_expression.

This executes the operations: DIVIDED_BY, MODULO, POWER, TIMES.

*/

int Interp::execute_binary1(double *left,        //!< pointer to the left operand    
                           int operation,       //!< integer code for the operation 
                           double *right)       //!< pointer to the right operand   
{
  static char name[] = "execute_binary1";
  switch (operation) {
  case DIVIDED_BY:
    CHK((*right == 0.0), NCE_ATTEMPT_TO_DIVIDE_BY_ZERO);
    *left = (*left / *right);
    break;
  case MODULO:                 /* always calculates a positive answer */
    *left = fmod(*left, *right);
    if (*left < 0.0) {
      *left = (*left + fabs(*right));
    }
    break;
  case POWER:
    CHK(((*left < 0.0) && (floor(*right) != *right)),
        NCE_ATTEMPT_TO_RAISE_NEGATIVE_TO_NON_INTEGER_POWER);
    *left = pow(*left, *right);
    break;
  case TIMES:
    *left = (*left * *right);
    break;
  default:
    ERM(NCE_BUG_UNKNOWN_OPERATION);
  }
  return RS274NGC_OK;
}

/****************************************************************************/

/*! execute_binary2

Returned Value: int
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns RS274NGC_OK.
   1. operation is unknown: NCE_BUG_UNKNOWN_OPERATION

Side effects:
   The result from performing the operation is put into what left points at.

Called by: read_real_expression.

This executes the operations: AND2, EXCLUSIVE_OR, MINUS,
NON_EXCLUSIVE_OR, PLUS. The RS274/NGC manual [NCMS] does not say what
the calculated value of the three logical operations should be. This
function calculates either 1.0 (meaning true) or 0.0 (meaning false).
Any non-zero input value is taken as meaning true, and only 0.0 means
false.


*/

int Interp::execute_binary2(double *left,        //!< pointer to the left operand    
                           int operation,       //!< integer code for the operation 
                           double *right)       //!< pointer to the right operand   
{
  static char name[] = "execute_binary2";
  switch (operation) {
  case AND2:
    *left = ((*left == 0.0) || (*right == 0.0)) ? 0.0 : 1.0;
    break;
  case EXCLUSIVE_OR:
    *left = (((*left == 0.0) && (*right != 0.0))
             || ((*left != 0.0) && (*right == 0.0))) ? 1.0 : 0.0;
    break;
  case MINUS:
    *left = (*left - *right);
    break;
  case NON_EXCLUSIVE_OR:
    *left = ((*left != 0.0) || (*right != 0.0)) ? 1.0 : 0.0;
    break;
  case PLUS:
    *left = (*left + *right);
    break;
  default:
    ERM(NCE_BUG_UNKNOWN_OPERATION);
  }
  return RS274NGC_OK;
}

/****************************************************************************/

/*! execute_block

Returned Value: int
   If convert_stop returns RS274NGC_EXIT, this returns RS274NGC_EXIT.
   If any of the following functions is called and returns an error code,
   this returns that code.
     convert_comment
     convert_feed_mode
     convert_feed_rate
     convert_g
     convert_m
     convert_speed
     convert_stop
     convert_tool_select
   Otherwise, if the probe_flag in the settings is ON, this returns
      RS274NGC_EXECUTE_FINISH.
   Otherwise, it returns RS274NGC_OK.

Side effects:
   One block of RS274/NGC instructions is executed.

Called by:
   rs274ngc_execute

This converts a block to zero to many actions. The order of execution
of items in a block is critical to safe and effective machine operation,
but is not specified clearly in the RS274/NGC documentation.

Actions are executed in the following order:
1. any comment.
2. a feed mode setting (g93, g94)
3. a feed rate (f) setting if in units_per_minute feed mode.
4. a spindle speed (s) setting.
5. a tool selection (t).
6. "m" commands as described in convert_m (includes tool change).
7. any g_codes (except g93, g94) as described in convert_g.
8. stopping commands (m0, m1, m2, m30, or m60).

In inverse time feed mode, the explicit and implicit g code executions
include feed rate setting with g1, g2, and g3. Also in inverse time
feed mode, attempting a canned cycle cycle (g81 to g89) or setting a
feed rate with g0 is illegal and will be detected and result in an
error message.

*/

int Interp::execute_block(block_pointer block,   //!< pointer to a block of RS274/NGC instructions
                         setup_pointer settings)        //!< pointer to machine settings                 
{
  static char name[] = "execute_block";
  int status;

  if (block->comment[0] != 0) {
    CHP(convert_comment(block->comment));
  }
  if (block->g_modes[5] != -1) {
    CHP(convert_feed_mode(block->g_modes[5], settings));
  }
  if (block->f_number > -1.0) {
    if (settings->feed_mode == INVERSE_TIME);   /* handle elsewhere */
    else {
      CHP(convert_feed_rate(block, settings));
    }
  }
  if (block->s_number > -1.0) {
    CHP(convert_speed(block, settings));
  }
  if (block->t_number != -1) {
    CHP(convert_tool_select(block, settings));
  }
  CHP(convert_m(block, settings));
  CHP(convert_g(block, settings));
  if (block->m_modes[4] != -1) {        /* converts m0, m1, m2, m30, or m60 */
    status = convert_stop(block, settings);
    if (status == RS274NGC_EXIT)
      return RS274NGC_EXIT;
    else if (status != RS274NGC_OK)
      ERP(status);
  }
  return ((settings->probe_flag ==
           ON) ? RS274NGC_EXECUTE_FINISH : RS274NGC_OK);
}

/****************************************************************************/

/*! execute_unary

Returned Value: int
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns RS274NGC_OK.
   1. the operation is unknown: NCE_BUG_UNKNOWN_OPERATION
   2. the argument to acos is not between minus and plus one:
      NCE_ARGUMENT_TO_ACOS_OUT_RANGE
   3. the argument to asin is not between minus and plus one:
      NCE_ARGUMENT_TO_ASIN_OUT_RANGE
   4. the argument to the natural logarithm is not positive:
      NCE_ZERO_OR_NEGATIVE_ARGUMENT_TO_LN
   5. the argument to square root is negative:
      NCE_NEGATIVE_ARGUMENT_TO_SQRT

Side effects:
   The result from performing the operation on the value in double_ptr
   is put into what double_ptr points at.

Called by: read_unary.

This executes the operations: ABS, ACOS, ASIN, COS, EXP, FIX, FUP, LN
ROUND, SIN, SQRT, TAN

All angle measures in the input or output are in degrees.

*/

int Interp::execute_unary(double *double_ptr,    //!< pointer to the operand         
                         int operation) //!< integer code for the operation 
{
  static char name[] = "execute_unary";
  switch (operation) {
  case ABS:
    if (*double_ptr < 0.0)
      *double_ptr = (-1.0 * *double_ptr);
    break;
  case ACOS:
    CHK(((*double_ptr < -1.0) || (*double_ptr > 1.0)),
        NCE_ARGUMENT_TO_ACOS_OUT_OF_RANGE);
    *double_ptr = acos(*double_ptr);
    *double_ptr = ((*double_ptr * 180.0) / PI);
    break;
  case ASIN:
    CHK(((*double_ptr < -1.0) || (*double_ptr > 1.0)),
        NCE_ARGUMENT_TO_ASIN_OUT_OF_RANGE);
    *double_ptr = asin(*double_ptr);
    *double_ptr = ((*double_ptr * 180.0) / PI);
    break;
  case COS:
    *double_ptr = cos((*double_ptr * PI) / 180.0);
    break;
  case EXP:
    *double_ptr = exp(*double_ptr);
    break;
  case FIX:
    *double_ptr = floor(*double_ptr);
    break;
  case FUP:
    *double_ptr = ceil(*double_ptr);
    break;
  case LN:
    CHK((*double_ptr <= 0.0), NCE_ZERO_OR_NEGATIVE_ARGUMENT_TO_LN);
    *double_ptr = log(*double_ptr);
    break;
  case ROUND:
    *double_ptr = (double)
      ((int) (*double_ptr + ((*double_ptr < 0.0) ? -0.5 : 0.5)));
    break;
  case SIN:
    *double_ptr = sin((*double_ptr * PI) / 180.0);
    break;
  case SQRT:
    CHK((*double_ptr < 0.0), NCE_NEGATIVE_ARGUMENT_TO_SQRT);
    *double_ptr = sqrt(*double_ptr);
    break;
  case TAN:
    *double_ptr = tan((*double_ptr * PI) / 180.0);
    break;
  default:
    ERM(NCE_BUG_UNKNOWN_OPERATION);
  }
  return RS274NGC_OK;
}

