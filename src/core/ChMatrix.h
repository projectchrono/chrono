#ifndef CHMATRIX_H
#define CHMATRIX_H

//////////////////////////////////////////////////
//  
//   ChMatrix.h
//
//   Math functions for :
//      - MATRICES
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright 1996/2005 Alessandro Tasora 
// ------------------------------------------------
///////////////////////////////////////////////////



#include "core/ChCoordsys.h"
#include "core/ChStream.h"
#include "core/ChException.h"

namespace chrono
{



//
// FAST MACROS TO SPEEDUP CODE
//


#define SetZero(els) {for (int i=0; i<els; ++i) this->address[i]=0; }
#define ElementsCopy(to,from,els) {for (int i=0; i<els; ++i) to[i]=(Real)from[i]; }

#define Set33Element(a,b,val)  SetElementN(((a*3)+(b)),val) 
#define Get33Element(a,b)      GetElementN((a*3)+(b)) 

#define Set34Element(a,b,val)  SetElementN(((a*4)+(b)),val) 
#define Get34Element(a,b)      GetElementN((a*4)+(b)) 
#define Set34Row(ma,a,val0,val1,val2,val3)  ma.SetElementN((a*4),val0);ma.SetElementN((a*4)+1,val1);ma.SetElementN((a*4)+2,val2);ma.SetElementN((a*4)+3,val3);

#define Set44Element(a,b,val)  SetElementN(((a*4)+(b)),val) 
#define Get44Element(a,b)      GetElementN((a*4)+(b)) 

//forward declaration
template <class Real = double> class ChMatrixDynamic ;

#define CHCLASS_MATRIX 14

///
/// ChMatrix:		
/// 
///  A base class for matrix objects (tables of NxM numbers).
/// To access elements, the indexes start from zero, and
/// you must indicate first row, then column, that is: m(2,4)
/// means the element at 3rd row, 5th column.
///  This is an abstract class, so you cannot instantiate 
/// objects from it: you must rather create matrices using the
/// specialized child classes like ChMatrixDynamic, ChMatrixNM
/// ChMatrix33 and so on; all of them have this same base class.
///  Warning: for optimization reasons, not all functions will
/// check about boundaries of element indexes and matrix sizes (in
/// some cases, if sizes are wrong, debug asserts are used).


template <class Real = double>
class ChMatrix 
{
protected:

			//
			// DATA
			//

	int rows;
	int	columns;
	Real* address;

public:

			//
			// CONSTRUCTORS (none - abstract class that must be implemented with child classes)
			//

	virtual ~ChMatrix () {};


			//
			// OPERATORS OVERLOADING
			//

					/// Parenthesis () operator, to access a single element of the matrix, by
					/// supplying the row and the column (indexes start from 0).
					/// For example: m(3,5) gets the element at the 4th row, 6th column.
					/// Value is returned by reference, so it can be modified, like in m(1,2)=10.
	inline Real& operator()(const int row, const int col) 
						{	
							assert (row >= 0 && col >= 0 && row < rows && col < columns);	 
							return (*(address + col +(row*columns)));
						};
	inline const Real& operator()(const int row,  const int col) const
						{	
							assert (row >= 0 && col >= 0 && row < rows && col < columns);	 
							return (*(address + col +(row*columns)));
						};

					/// Parenthesis () operator, to access a single element of the matrix, by
					/// supplying the ordinal of the element (indexes start from 0).
					/// For example: m(3) gets the 4th element, counting row by row. 
					/// Mostly useful if the matrix is Nx1 sized (i.e. a N-element vector).
					/// Value is returned by reference, so it can be modified, like in m(1,2)=10.
	inline Real& operator()(const int el) 
						{	
							assert (el >= 0 && el < rows*columns);	 
							return (*(address + el));
						};
	inline const Real& operator()(const int el) const
						{	
							assert (el >= 0 && el < rows*columns);	 
							return (*(address + el));
						};

					/// The [] operator returns the address of the n-th row. This is mostly
					/// for compatibility with old matrix programming styles (2d array-like) 
					/// where to access an element at row i, column j, one can write mymatrix[i][j]. 
	inline Real* operator[](const int row) 
						{	
							assert (row >= 0 && row < rows);	 
							return ((address +(row*columns)));
						};
	inline const Real* operator[](const int row) const 
						{	
							assert (row >= 0 && row < rows);	 
							return ((address +(row*columns)));
						};


					///	Multiplies this matrix by a factor, in place
	ChMatrix<Real>& operator*=(const Real factor) 
						{ MatrScale (factor); return *this;};
	
					///	Increments this matrix by another matrix, in place
	template <class RealB>
	ChMatrix<Real>& operator+=(const ChMatrix<RealB>& matbis) 
						{ MatrInc (matbis); return *this;};
	
					///	Decrements this matrix by another matrix, in place
	template <class RealB>
	ChMatrix<Real>& operator-=(const ChMatrix<RealB>& matbis) 
						{ MatrDec (matbis); return *this;};

					/// Matrices are equal?
	bool operator==(const ChMatrix<Real>& other)  { return Equals(other);}
					/// Matrices are not equal?
	bool operator!=(const ChMatrix<Real>& other)  { return !Equals(other);}

					/// Assignment operator
	inline	ChMatrix<Real>&	operator =(const ChMatrix<Real>& matbis)
						{
							if (&matbis != this) 
								CopyFromMatrix(matbis);
							return *this;
						}
	template <class RealB>
	inline	ChMatrix<Real>&	operator =(const ChMatrix<RealB>& matbis)
						{ 
							CopyFromMatrix(matbis);
							return *this;
						}

			//
			// FUNCTIONS
			//

					/// Sets the element at row,col position. Indexes start with zero.
	inline void   SetElement ( int row, int col, Real elem)
						{ 
							assert (row >= 0 && col >= 0 && row < rows && col < columns);	 // boundary checks
							*(address + col +(row*columns)) = elem; 
						};
					/// Gets the element at row,col position. Indexes start with zero.
					/// The return value is a copy of original value. Use Element() instead if you
					/// want to access directly by reference the original element.
	inline Real  GetElement ( int row, int col)
						{
							assert (row >= 0 && col >= 0 && row < rows && col < columns);	 // boundary checks
							return(*(address + col +(row*columns))); 
						};
	inline Real  GetElement ( int row, int col) const
						{
							assert (row >= 0 && col >= 0 && row < rows && col < columns);	 // boundary checks
							return(*(address + col +(row*columns))); 
						};
					/// Sets the Nth element, counting row after row.
	inline void   SetElementN (int index, Real elem)
						{
							assert (index >=0 && index < (rows*columns));	// boundary checks
							*(address+index) = elem;
						}
					/// Gets the Nth element, counting row after row. 
	inline Real   GetElementN (int index)
						{
							assert (index >=0 && index < (rows*columns));
							return(*(address+index));
						}
	inline const Real GetElementN (int index) const
						{
							assert (index >=0 && index < (rows*columns));
							return(*(address+index));
						}

					/// Access a single element of the matrix, by
					/// supplying the row and the column (indexes start from 0).
					/// Value is returned by reference, so it can be modified, like in m.Element(1,2)=10.
 	inline Real& Element (int row, int col)
						{
							assert (row >= 0 && col >= 0 && row < rows && col < columns);
							return(*(address + col +(row*columns)));
						}
 	inline const Real& Element (int row, int col) const
						{
							assert (row >= 0 && col >= 0 && row < rows && col < columns);
							return(*(address + col +(row*columns)));
						}
					/// Access a single element of the matrix, the Nth element, counting row after row.
					/// Value is returned by reference, so it can be modified, like in m.Element(5)=10.
 	inline Real& ElementN (int index)
						{
							assert (index >=0 && index < (rows*columns));
							return(*(address+index));
						}
 	inline const Real& ElementN (int index) const
						{
							assert (index >=0 && index < (rows*columns));
							return(*(address+index));
						}

					/// Access directly the "Real* address" buffer. Warning! this is a low level
					/// function, it should be used in rare cases, if really needed!
	inline Real* GetAddress ()  { return address; };
	inline const Real* GetAddress () const { return address; };

					/// Gets the number of rows
	inline int GetRows ()	const	{ return rows; }
				
					/// Gets the number of columns
	inline int GetColumns () const	{ return columns; }

					/// Reallocate memory for a new size. VIRTUAL! Must be implemented by child classes!
	virtual inline void Resize(int nrows, int ncols)=0;

					/// Swaps the columns a and b
	void SwapColumns (int a, int b)
						{
							Real temp;
							for (int i=0; i < rows; i++)
							{
								temp = GetElement (i,a);
								SetElement (i,a, GetElement (i,b));
								SetElement (i,b, temp);
							}
						}

					/// Swap the rows a and b
	void SwapRows    (int a, int b)
						{
							Real temp; 
							for (int i=0; i < columns; i++)
							{
								temp = GetElement (a,i);
								SetElement (a,i, GetElement (b,i));
								SetElement (b,i, temp);
							}
						}

					/// Fill the diagonal elements, given a sample.
					/// Note that the matrix must already be square (no check for 
					/// rectangular matrices!), and the extradiagonal elements are
					/// not modified -this function does not set them to 0-
	void FillDiag	(Real sample)
						{
							for (int i=0;i < rows; ++i)
								SetElement (i,i,sample);
						}

					/// Fill the matrix with the same value in all elements
	void FillElem	(Real sample)
						{
							for (int i=0;i < rows*columns; ++i)
								SetElementN (i,sample);
						}

					/// Fill the matrix with random float numbers, falling within the
					/// "max"/"min" range.
	void FillRandom	(Real max, Real min)
						{
							for (int i=0;i < rows*columns; ++i)
								SetElementN (i, min+ ChRandom()*(max-min));
						}

					/// Resets the matrix to zero  (warning: simply sets memory to 0 bytes!)
	void Reset()		{   
							SetZero(rows*columns); //memset(address, 0, sizeof(Real) * rows * columns); 
						}	
					
					/// Reset to zeroes and (if needed) changes the size to have row and col
	void Reset(int nrows, int ncols)
						{
							Resize(nrows, ncols);
							SetZero(rows*columns); //memset(address, 0, sizeof(Real) * rows * columns);
						}
					
					/// Reset to identity matrix (ones on diagonal, zero elsewhere)
	void SetIdentity()  {   Reset(); FillDiag(1.0); }

	
					/// Copy a matrix "matra" into this matrix. Note that 
					/// the destination matrix will be resized if necessary.
	template <class RealB>
	void CopyFromMatrix (ChMatrix<RealB>& matra)
						{
							Resize(matra.GetRows(), matra.GetColumns());
							ElementsCopy(address, matra.GetAddress(), rows*columns); //memcpy (address, matra.address, (sizeof(Real) * rows * columns));
						}
	template <class RealB>
	void CopyFromMatrix (const ChMatrix<RealB>& matra)
						{
							Resize(matra.GetRows(), matra.GetColumns());
							ElementsCopy(address, matra.GetAddress(), rows*columns); //memcpy (address, matra.address, (sizeof(Real) * rows * columns));
						}

					/// Copy the transpose of matrix "matra" into this matrix. Note that 
					/// the destination matrix will be resized if necessary.
	template <class RealB>
	void CopyFromMatrixT (ChMatrix<RealB>& matra)
						{
							Resize(matra.GetColumns(), matra.GetRows());
							for (int i=0;i < matra.GetRows(); ++i)
								for (int j=0;j < matra.GetColumns(); ++j)
									SetElement (j,i,(matra.Element(i,j)));
						}
	

			//
			// STREAMING
			//
					/// Method to allow serializing transient data into in ascii
					/// as a readable item, for example   "chrono::GetLog() << myobject;"
	void StreamOUT(ChStreamOutAscii& mstream)
						{
							mstream << "\n" << "Matrix " << GetRows() << " rows, " << GetColumns() << " columns." << "\n";
							for (int i=0;i < ChMin(GetRows(),8); i++)
							{
									for (int j=0;j < ChMin(GetColumns(),8); j++)
										mstream << GetElement (i,j) <<"  ";
									if (GetColumns()>8) mstream << "...";
									mstream << "\n";
							}
							if (GetRows()>8) mstream << "... \n\n";
						}

					/// Method to allow serializing transient data into an ascii stream (ex. a file)
					/// as a Matlab .dat file (all numbers in a row, separated by space, then CR)
	void StreamOUTdenseMatlabFormat(ChStreamOutAscii& mstream)
						{
							for(int ii=0; ii<this->GetRows(); ii++)
							{
								for(int jj=0; jj<this->GetColumns(); jj++)
								{
									mstream << this->GetElement(ii,jj);
									if (jj<(this->GetColumns()-1))
										mstream << " ";
								}
								mstream << "\n";
							}
						}

					/// Method to allow serializing transient data into a persistent
					/// binary archive (ex: a file).
	void StreamOUT(ChStreamOutBinary& mstream)
						{
							mstream << GetRows();
							mstream << GetColumns();
							int tot_elements = GetRows() * GetColumns();
							for (int i=0; i< tot_elements; i++)
							{
								mstream << GetElementN(i);
							}
						}

					/// Method to allow deserializing a persistent binary archive (ex: a file)
					/// into transient data.
	void StreamIN(ChStreamInBinary& mstream)
						{
							int m_row, m_col;
							mstream >> m_row;
							mstream >> m_col;
							if ((m_row ==0)||(m_col==0))
								throw (ChException("Cannot load zero-sized matrix: wrong stream format?"));
							Reset(m_row, m_col);
							for (int i=0; i< (m_row*m_col); i++)
							{
								mstream >> ElementN(i);
							}
						}




				//
				// Math member functions. For speed reasons, sometimes 
				// size checking of operands is left to the user!
				//

					/// Changes the sign of all the elements of this matrix, in place.
	void MatrNeg()	{  for (int nel=0; nel<rows*columns; ++nel)	ElementN(nel)= -ElementN(nel);	}

	
					/// Sum two matrices, and stores the result in "this" matrix
					/// [this]=[A]+[B]. 
	template <class RealB,class RealC>
	void MatrAdd	( const ChMatrix<RealB>& matra, const ChMatrix<RealC>& matrb)
						{
							assert (matra.GetColumns()==matrb.GetColumns() && matra.rows==matrb.GetRows());
							assert (this->columns==matrb.GetColumns() && this->rows==matrb.GetRows());
							for (int nel=0; nel<rows*columns; ++nel)
								ElementN(nel)= matra.ElementN(nel)+matrb.ElementN(nel);
						}

					/// Subtract two matrices, and stores the result in "this" matrix
					/// [this]=[A]-[B]. 
	template <class RealB,class RealC>
	void MatrSub	( const ChMatrix<RealB>& matra, const ChMatrix<RealC>& matrb)
						{
							assert (matra.GetColumns()==matrb.GetColumns() && matra.rows==matrb.GetRows());
							assert (this->columns==matrb.GetColumns() && this->rows==matrb.GetRows());
							for (int nel=0; nel<rows*columns; ++nel)
								ElementN(nel)= matra.ElementN(nel)-matrb.ElementN(nel);
						}

					/// Increments this matrix with another matrix A, as: [this]+=[A]
	template <class RealB>
	void MatrInc	( const ChMatrix<RealB>& matra)
						{
							assert (matra.GetColumns()==columns && matra.GetRows()==rows);
							for (int nel=0; nel<rows*columns; ++nel)
								ElementN(nel)+= (Real)matra.ElementN(nel);
						}

					/// Decrements this matrix with another matrix A, as: [this]-=[A]
	template <class RealB>
	void MatrDec	( const ChMatrix<RealB>& matra)
						{
							assert (matra.GetColumns()==columns && matra.GetRows()==rows);
							for (int nel=0; nel<rows*columns; ++nel)
								ElementN(nel)-= (Real)matra.ElementN(nel);
						}

					/// Scales a matrix, multiplying all elements by a constant value: [this]*=f
	void MatrScale	(Real factor)
						{
							for (int nel=0; nel<rows*columns; ++nel)
								ElementN(nel)*= factor;
						}
					
					/// Multiplies two matrices, and stores the result in "this" matrix
					/// [this]=[A]*[B].  
	template <class RealB,class RealC>
	void MatrMultiply  ( const ChMatrix<RealB>& matra, const ChMatrix<RealC>& matrb)
						{
							assert (matra.GetColumns()==matrb.GetRows());	
							assert (this->rows==matra.GetRows());
							assert (this->columns==matrb.GetColumns());
							register int col, row, colres; 	
							Real sum;
							for (colres=0; colres < matrb.GetColumns(); ++colres)
								for (row=0; row < matra.GetRows(); ++row)
								{
									sum = 0;
									for (col=0; col < matra.GetColumns(); ++col)
										sum+= (matra.Element(row,col))*(matrb.Element(col,colres));
									SetElement (row, colres, sum);
								}
						}

					/// Multiplies two matrices (the second is considered transposed): 
					/// [this]=[A]*[B]'
					/// Faster than doing B.MatrTranspose(); result.MatrMultiply(A,B); 
					/// Note: no check on mistaken size of this! 
	template <class RealB,class RealC>
	void MatrMultiplyT ( const ChMatrix<RealB>& matra, const ChMatrix<RealC>& matrb)
						{
							assert (matra.GetColumns()==matrb.GetColumns());
							assert (this->rows==matra.GetRows());
							assert (this->columns==matrb.GetRows());
							register int col, row, colres; 	
							Real sum;
							for (colres=0; colres < matrb.GetRows(); ++colres)
								for (row=0; row < matra.GetRows(); ++row)
								{
									sum = 0;
									for (col=0; col < matra.GetColumns(); ++col)
										sum+= (matra.Element (row,col))*(matrb.Element(colres,col));
									SetElement (row, colres, sum);
								}
						}

					/// Multiplies two matrices (the first is considered transposed): 
					/// [this]=[A]'*[B]
					/// Faster than doing A.MatrTranspose(); result.MatrMultiply(A,B); 
	template <class RealB,class RealC>
	void MatrTMultiply ( const ChMatrix<RealB>& matra, const ChMatrix<RealC>& matrb)
						{
							assert (matra.GetRows()==matrb.GetRows());
							assert (this->rows==matra.GetColumns());
							assert (this->columns==matrb.GetColumns());
							register int col, row, colres; 	
							Real sum;
							for (colres=0; colres < matrb.GetColumns(); ++colres)
								for (row=0; row < matra.GetColumns(); ++row)
								{
									sum = 0;
									for (col=0; col < (matra.GetRows()); ++col)
										sum+= (matra.Element(col,row))*(matrb.Element(col,colres));
									SetElement (row, colres, sum);
								}
						}

					/// Computes dot product between two column-matrices (vectors) with 
					/// same size. Returns a scalar value.
	template <class RealB,class RealC>
	static Real MatrDot(const ChMatrix<RealB>* ma, const ChMatrix<RealC>* mb)
						{
							assert (ma->GetColumns()==mb->GetColumns() && ma->GetRows()==mb->GetRows());
							Real tot = 0;
							for (int i=0; i<ma->GetRows(); ++i)
								tot += ma->ElementN(i) * mb->ElementN(i);
							return tot;
						}

					/// Transpose this matrix in place
	void MatrTranspose	()
						{
							if (columns==rows)	// Square transp.is optimized
							{
								for (int row=0; row < rows; ++row)
									for (int col=row; col < columns; ++col)
										if (row!=col)
										{
											Real temp = Element (row,col);
											Element (row,col)= Element (col,row);
											Element (col,row)= temp;
										}
								int tmpr = rows; rows = columns; columns = tmpr;
							}
							else	// Naive implementation for rectangular case. Not in-place. Slower.
							{
								ChMatrixDynamic<Real> matrcopy(*this);
								int tmpr = rows; rows = columns; columns = tmpr;	// dont' realloc buffer, anyway
								for (int row=0; row < rows; ++row)
									for (int col=0; col < columns; ++col)
										Element(row,col)=matrcopy.Element(col,row);
							}
						}
			

					/// Returns true if vector is identical to other vector
	bool	Equals ( const ChMatrix<Real>& other) { return Equals(other,0.0); }

					/// Returns true if vector equals another vector, within a tolerance 'tol'
	bool	Equals ( const ChMatrix<Real>& other, Real tol) 
						{
							if ((other.GetColumns() != this->columns)||(other.GetRows() != this->rows)) return false;
							for (int nel=0; nel<rows*columns; ++nel)
								if (fabs(ElementN(nel)-other.ElementN(nel)) > tol) return false;
							return true;
						}



					/// Multiplies this 3x4 matrix by a quaternion, as v=[G]*q
					/// The matrix must be 3x4.
					///  \return The result of the multiplication, i.e. a vector.
	template <class RealB>
	ChVector<Real> Matr34_x_Quat (ChQuaternion<RealB> qua)
						{
							assert ((rows ==3) && (columns ==4));
							return ChVector<Real> (
										((Get34Element(0,0))*qua.e0)+
									    ((Get34Element(0,1))*qua.e1)+
									    ((Get34Element(0,2))*qua.e2)+
									    ((Get34Element(0,3))*qua.e3)  ,
										((Get34Element(1,0))*qua.e0)+
									    ((Get34Element(1,1))*qua.e1)+
									    ((Get34Element(1,2))*qua.e2)+
									    ((Get34Element(1,3))*qua.e3)  ,
										((Get34Element(2,0))*qua.e0)+
									    ((Get34Element(2,1))*qua.e1)+
									    ((Get34Element(2,2))*qua.e2)+
									    ((Get34Element(2,3))*qua.e3)  );
						}

					/// Multiplies this 3x4 matrix (transposed) by a vector, as q=[G]'*v
					/// The matrix must be 3x4.
					///  \return The result of the multiplication, i.e. a quaternion.
	template <class RealB>
	ChQuaternion<Real> Matr34T_x_Vect (ChVector<RealB> va)
						{
							assert ((rows ==3) && (columns ==4));
							return ChQuaternion<Real>  (
										((Get34Element(0,0))*va.x)+
										((Get34Element(1,0))*va.y)+
										((Get34Element(2,0))*va.z)  ,
										((Get34Element(0,1))*va.x)+
									    ((Get34Element(1,1))*va.y)+
									    ((Get34Element(2,1))*va.z)  ,
										((Get34Element(0,2))*va.x)+
										((Get34Element(1,2))*va.y)+
										((Get34Element(2,2))*va.z)  ,
										((Get34Element(0,3))*va.x)+
										((Get34Element(1,3))*va.y)+
										((Get34Element(2,3))*va.z)  );
						}

					/// Multiplies this 4x4 matrix (transposed) by a quaternion, 
					/// The matrix must be  4x4.
					///  \return The result of the multiplication, i.e. a quaternion.
	template <class RealB>
	ChQuaternion<Real> Matr44_x_Quat (ChQuaternion<RealB> qua)
						{
							assert ((rows ==4) && (columns ==4));
							return ChQuaternion<Real> (
										((Get44Element(0,0))*qua.e0)+
										((Get44Element(0,1))*qua.e1)+
										((Get44Element(0,2))*qua.e2)+
										((Get44Element(0,3))*qua.e3)  ,
										((Get44Element(1,0))*qua.e0)+
										((Get44Element(1,1))*qua.e1)+
										((Get44Element(1,2))*qua.e2)+
										((Get44Element(1,3))*qua.e3)  ,
										((Get44Element(2,0))*qua.e0)+
										((Get44Element(2,1))*qua.e1)+
										((Get44Element(2,2))*qua.e2)+
										((Get44Element(2,3))*qua.e3)  ,
										((Get44Element(3,0))*qua.e0)+
										((Get44Element(3,1))*qua.e1)+
										((Get44Element(3,2))*qua.e2)+
										((Get44Element(3,3))*qua.e3)  );
						}

					/// Transposes only the lower-right 3x3 submatrix of a hemisimetric 4x4 matrix,
					/// used when the 4x4 matrix is a "star" matrix [q] coming from a quaternion q:
					/// the non commutative quat. product is: 
					///     q1 x q2  =  [q1]*q2  =  [q2st]*q1 
					/// where [q2st] is the "semitranspose of [q2].
	void MatrXq_SemiTranspose()
						{
							SetElement (1,2, -GetElement (1,2));
							SetElement (1,3, -GetElement (1,3));
							SetElement (2,1, -GetElement (2,1));
							SetElement (2,3, -GetElement (2,3));
							SetElement (3,1, -GetElement (3,1));
							SetElement (3,2, -GetElement (3,2));
						}

					/// Change the sign of the 2nd, 3rd and 4th columns of a 4x4 matrix,
					/// The product between a quaternion q1 and the coniugate of q2 (q2'), is:
					///    q1 x q2'  = [q1]*q2'   = [q1sn]*q2
					/// where [q1sn] is the seminegation of the 4x4 matrix [q1]. 
	void MatrXq_SemiNeg()
						{
							for (int i=0;i < rows;++i)
								for (int j=1;j < columns;++j)
									SetElement (i,j,-GetElement(i,j));
						}


					/// Gets the norm infinite of the matrix, i.e. the max.
					/// of its elements in absolute value.
	Real NormInf ()
						{
							Real norm = 0;
							for (int nel=0; nel<rows*columns; ++nel)
								if ( (fabs (ElementN(nel))) > norm)
									norm= fabs (ElementN(nel));
							return norm;
						}

					/// Gets the norm two of the matrix, i.e. the square root
					/// of the sum of the elements squared.
	Real NormTwo ()
						{
							Real norm = 0;
								for (int nel=0; nel<rows*columns; ++nel)	
									norm += ElementN(nel)*ElementN(nel);
							return (sqrt(norm));
						}


					/// Finds max value among the values of the matrix
	Real Max()			{
							Real mmax = GetElement(0,0);
							for (int nel=0; nel<rows*columns; ++nel)
								if (ElementN(nel)>mmax) mmax = ElementN(nel);
							return mmax;
						}
					/// Finds min value among the values of the matrix
	Real Min()			{
							Real mmin = GetElement(0,0);
							for (int nel=0; nel<rows*columns; ++nel)
								if (ElementN(nel)<mmin) mmin = ElementN(nel);
							return mmin;
						}

					/// Linear interpolation of two matrices. Parameter mx must be 0...1.
					/// [this] =(1-x)[A]+ (x)[B]    Matrices must have the same size!!
	void LinInterpolate(const ChMatrix<Real>& matra, const ChMatrix<Real>& matrb, double mx)
						{
							assert (matra.columns==matrb.columns && matra.rows==matrb.rows);
							for (int nel=0; nel<rows*columns; nel++)
								ElementN(nel)= matra.ElementN(nel)*(1.0-mx) + matrb.ElementN(nel)*(mx);
						}

					/// Fills a matrix or a vector with a bilinear interpolation, 
					/// from corner values (as a u-v patch).
	void RowColInterp(double vmin, double vmax, double umin, double umax)
						{
							for (int iu = 0; iu <GetColumns(); iu++)
								for (int iv = 0; iv <GetRows(); iv++)
								{
									if (GetRows()>1)    Element(iv,iu) =  vmin+(vmax-vmin)*((double)iv/((double)(GetRows()-1)));
									if (GetColumns()>1)	Element(iv,iu) += umin+(umax-umin)*((double)iu/((double)(GetColumns()-1)));
								}
						}

			//
			// BOOKKEEPING
			//


					/// Paste a matrix "matra" into "this", inserting at location insrow-inscol.
					/// Normal copy for insrow=inscol=0 
	template <class RealB>
	void PasteMatrix (ChMatrix<RealB>* matra, int insrow, int inscol)
						{
							for (int i=0;i < matra->GetRows();++i)
								for (int j=0;j < matra->GetColumns();++j)
									Element(i+insrow, j+inscol) = matra->Element (i,j);
						}

					/// Paste a matrix "matra" into "this", inserting at location insrow-inscol
					/// and performing a sum with the preexisting values.
	template <class RealB>
	void PasteSumMatrix (ChMatrix<RealB>* matra, int insrow, int inscol)
						{
							for (int i=0;i < matra->GetRows();++i)
								for (int j=0;j < matra->GetColumns();++j)
									Element(i+insrow, j+inscol) += matra->Element (i,j);
						}

					/// Paste a matrix "matra", transposed, into "this", inserting at location insrow-inscol.
					/// Normal copy for insrow=inscol=0 
	template <class RealB>
	void PasteTranspMatrix (ChMatrix<RealB>* matra, int insrow, int inscol)
						{
							for (int i=0;i <  matra->GetRows(); ++i)
								for (int j=0;j < matra->GetColumns(); ++j)
									Element(j+insrow, i+inscol) = matra->Element (i,j);
						}

					/// Paste a matrix "matra", transposed, into "this", inserting at location insrow-inscol
					/// and performing a sum with the preexisting values.
	template <class RealB>
	void PasteSumTranspMatrix (ChMatrix<RealB>* matra, int insrow, int inscol)
						{
							for (int i=0;i <  matra->GetRows(); ++i)
								for (int j=0;j < matra->GetColumns(); ++j)
									Element(j+insrow, i+inscol) += matra->Element (i,j);
						}

					/// Paste a clipped portion of the matrix "matra" into "this",
					/// inserting the clip (of size nrows, ncolumns) at the location insrow-inscol.
	template <class RealB>
	void PasteClippedMatrix (ChMatrix<RealB>* matra, int cliprow, int clipcol, int nrows, int ncolumns, int insrow, int inscol)
						{
							for (int i=0;i < nrows;++i)
								for (int j=0;j < ncolumns;++j)
									Element (i+insrow, j+inscol) = (Real)matra->Element (i+cliprow,j+clipcol);
						}

					/// Paste a clipped portion of the matrix "matra" into "this", performing a sum with preexisting values,
					/// inserting the clip (of size nrows, ncolumns) at the location insrow-inscol.
	template <class RealB>
	void PasteSumClippedMatrix (ChMatrix<RealB>* matra, int cliprow, int clipcol, int nrows, int ncolumns, int insrow, int inscol)
						{
							for (int i=0;i < nrows;++i)
								for (int j=0;j < ncolumns;++j)
									Element (i+insrow, j+inscol) += (Real)matra->Element (i+cliprow,j+clipcol);
						}

					/// Paste a vector "va" into the matrix.
	template <class RealB>
	void PasteVector (ChVector<RealB> va, int insrow, int inscol)
						{
							SetElement (insrow,   inscol, (Real)va.x);
							SetElement (insrow+1, inscol, (Real)va.y);
							SetElement (insrow+2, inscol, (Real)va.z);
						}


					/// Paste a vector "va" into the matrix, summing it with preexisting values..
	template <class RealB>
	void PasteSumVector (ChVector<RealB> va, int insrow, int inscol)
						{
							Element (insrow,   inscol) += va.x;
							Element (insrow+1, inscol) += va.y;
							Element (insrow+2, inscol) += va.z;
						}

					/// Paste a vector "va" into the matrix, subtracting it with preexisting values..
	template <class RealB>
	void PasteSubVector (ChVector<RealB> va, int insrow, int inscol)
						{
							Element (insrow,   inscol) -= va.x;
							Element (insrow+1, inscol) -= va.y;
							Element (insrow+2, inscol) -= va.z;
						}

					/// Paste a quaternion into the matrix.
	template <class RealB>
	void PasteQuaternion (ChQuaternion<RealB> qa, int insrow, int inscol)
						{
							SetElement (insrow,   inscol, qa.e0);
							SetElement (insrow+1, inscol, qa.e1);
							SetElement (insrow+2, inscol, qa.e2);
							SetElement (insrow+3, inscol, qa.e3);
						}

					/// Paste a quaternion into the matrix, summing it with preexisting values..
	template <class RealB>
	void PasteSumQuaternion (ChQuaternion<RealB> qa, int insrow, int inscol)
						{
							Element (insrow,   inscol) += qa.e0;
							Element (insrow+1, inscol) += qa.e1;
							Element (insrow+2, inscol) += qa.e2;
							Element (insrow+3, inscol) += qa.e3;
						}

					/// Paste a coordsys into the matrix.
	template <class RealB>
	void PasteCoordsys (ChCoordsys<RealB> cs, int insrow, int inscol)
						{
							PasteVector		(cs.pos, insrow, inscol);
							PasteQuaternion	(cs.rot, insrow+3 ,inscol);
						}

					/// Returns the vector clipped from insrow, inscol.
	ChVector<Real> ClipVector ( int insrow, int inscol)
						{	return ChVector<Real>(Element (insrow,inscol), Element (insrow+1,inscol), Element (insrow+2,inscol)); }

					/// Returns the quaternion clipped from insrow, inscol.
	ChQuaternion<Real> ClipQuaternion ( int insrow, int inscol)
						{	return ChQuaternion<Real>(Element (insrow,inscol), Element (insrow+1,inscol), Element (insrow+2,inscol), Element (insrow+3,inscol)); }

					/// Returns the coordsys clipped from insrow, inscol.
	ChCoordsys<Real> ClipCoordsys ( int insrow, int inscol)
						{	return ChCoordsys<Real>(ClipVector (insrow, inscol), ClipQuaternion (insrow+3, inscol)) ; }




			//
			// MULTIBODY SPECIFIC MATH FUCTION
			//

					/// Fills a 4x4 matrix as the "star" matrix, representing quaternion cross product.
					/// That is, given two quaternions a and b, aXb= [Astar]*b
	template <class RealB>
	void Set_Xq_matrix	( ChQuaternion<RealB> q )
	{
		Set44Element (0,0, q.e0 );
		Set44Element (0,1,-q.e1 );
		Set44Element (0,2,-q.e2 );
		Set44Element (0,3,-q.e3 );
		Set44Element (1,0, q.e1 );
		Set44Element (1,1, q.e0 );
		Set44Element (1,2,-q.e3 );
		Set44Element (1,3, q.e2 );
		Set44Element (2,0, q.e2 );
		Set44Element (2,1, q.e3 );
		Set44Element (2,2, q.e0 );
		Set44Element (2,3,-q.e1 );
		Set44Element (3,0, q.e3 );
		Set44Element (3,1,-q.e2 );
		Set44Element (3,2, q.e1 );
		Set44Element (3,3, q.e0 );
	}



};







///
/// ChMatrixDynamic
///
///  Specialized 'resizeable' matrix class where the elements are allocated on heap.
/// The size of the matrix can be known even at compile-time, and the matrix can 
/// be freely resized also after creation. The size is unlimited (until you have memory). 
///  Although this is the most generic type of matrix, please do not use it 
/// where you know in advance its size because there are more efficient
/// types for those matrices with 'static' size (for example, 3x3 rotation
/// matrices are faster if created as ChMatrix33).



template <class Real>
class ChMatrixDynamic : public ChMatrix<Real>
{
private:
			//
			// DATA
			//

			/// [simply use the  "Real* address" pointer of the base class

public:			
			//
			// CONSTRUCTORS
			//

									/// The default constructor builds a 3x3 matrix, 
	inline ChMatrixDynamic () 
						{ 
							this->rows= 3;
							this->columns = 3;
							this->address = new Real[9];
							SetZero(9);
						}	
									/// Copy constructor 
	inline ChMatrixDynamic (const ChMatrixDynamic<Real>& msource)
						{
							this->rows= msource.GetRows();
							this->columns= msource.GetColumns();
							this->address= new Real[this->rows*this->columns];
							ElementsCopy(this->address, msource.GetAddress(), this->rows*this->columns); //memcpy (address, msource.GetAddress(), (sizeof(Real) * rows * columns));
						}
									/// Copy constructor from all types of base matrices
	template <class RealB>
	inline ChMatrixDynamic (const ChMatrix<RealB>& msource)
						{
							this->rows= msource.GetRows();
							this->columns= msource.GetColumns();
							this->address= new Real[this->rows*this->columns];
							ElementsCopy(this->address, msource.GetAddress(), this->rows*this->columns); //memcpy (address, msource.GetAddress(), (sizeof(Real) * rows * columns));
						}
									/// The constructor for a generic nxm matrix. 
									/// Rows and columns cannot be zero or negative.
	inline ChMatrixDynamic (const int row, const int col )
						{
							assert (row >0 && col >0);
							this->rows= row;
							this->columns = col;
							this->address = new Real[row*col]; 
							SetZero(row*col);
						};


									/// Destructor
									/// Delete allocated heap mem. 
	virtual inline ~ChMatrixDynamic () 
						{ 
							delete[]this->address; 
						};

			//
			// OPERATORS
			//
					/// Assignment operator (from generic other matrix, it always work)
	inline	ChMatrixDynamic<Real>&	operator =(const ChMatrix<Real>& matbis)
						{
							ChMatrix<Real>::operator=(matbis);
							return *this;
						}

					///	Negates sign of the matrix. 
					/// Performance warning: a new object is created.
	ChMatrixDynamic<Real> operator-() 
						{ ChMatrixDynamic<Real> result (*this); result.MatrNeg(); return result;};

					///	Sums this matrix and another matrix. 
					/// Performance warning: a new object is created.
	template <class RealB>
	ChMatrixDynamic<Real> operator+(const ChMatrix<RealB>& matbis) 
						{ ChMatrixDynamic<Real> result (this->rows, this->columns); result.MatrAdd (*this, matbis); return result;};

					///	Subtracts this matrix and another matrix. 
					/// Performance warning: a new object is created.
	template <class RealB>
    ChMatrixDynamic<Real> operator-(const ChMatrix<RealB>& matbis) 
						{ ChMatrixDynamic<Real> result (this->rows, this->columns); result.MatrSub (*this, matbis); return result;};

					///	Multiplies this matrix and another matrix. 
					/// Performance warning: a new object is created.
	template <class RealB>
	ChMatrixDynamic<Real> operator*(const ChMatrix<RealB>& matbis)	
						{ ChMatrixDynamic<Real> result (this->rows, matbis.GetColumns()); result.MatrMultiply (*this, matbis); return result;};

					///	Multiplies this matrix by a scalar value.
					/// Performance warning: a new object is created.
	ChMatrixDynamic<Real> operator*(const Real factor) 
						{ ChMatrixDynamic<Real> result (*this); result.MatrScale (factor); return result;};


			//
			// FUNCTIONS
			//
									/// Reallocate memory for a new size.
	virtual inline void Resize(int nrows, int ncols)
						{
							assert (nrows > 0 && ncols > 0);
							if ((nrows != this->rows) || (ncols != this->columns))
							{
								this->rows= nrows;
								this->columns= ncols;
								delete[]this->address;		
								this->address= new Real[this->rows*this->columns]; 
								SetZero(this->rows*this->columns);
							}
						}
};




///
/// ChMatrixNM
///
/// Specialized matrix class having a pre-allocated NxM buffer of elements on stack.
///  This means that elements are put the stack when the matrix is created, and this
/// can be more efficient than heap allocation (but, please, do not use too large N or M 
/// sizes, this is meant to work up to 10x10, for example; prefer ChMatrixDyamic for larger).
///  The NxM size must be known 'statically', at compile-time; however later at run-time
/// this matrix can be resized anyway (but for larger size than NxM, it falls back to 
/// heap allocation). Note that if resizing is often required, it may be better
/// to create a ChMatrixDyamic instead, from the beginning.



template <class Real = double, int preall_rows=3, int preall_columns=3>
class ChMatrixNM : public ChMatrix<Real>
{
protected:
	Real  buffer[preall_rows*preall_columns];

public:			
			//
			// CONSTRUCTORS
			//

									/// The default constructor builds a NxN matrix, 
	inline ChMatrixNM () 
						{ 
							this->rows= preall_rows;
							this->columns = preall_columns;
							this->address = buffer;
							SetZero(preall_rows*preall_columns);
						}	
									/// Copy constructor
	inline ChMatrixNM (const ChMatrixNM<Real,preall_rows,preall_columns>& msource) 
						{
							this->rows= preall_rows;
							this->columns = preall_columns;
							this->address = buffer;
							ElementsCopy(this->address, msource.address, preall_rows*preall_columns); 
						}
									/// Copy constructor from all types of base matrices (only with same size)
	template <class RealB>
	inline ChMatrixNM (const ChMatrix<RealB>& msource) 
						{
							assert(msource.GetColumns()==preall_columns && msource.GetRows()==preall_rows);
							this->rows= preall_rows;
							this->columns = preall_columns;
							this->address = buffer;
							ElementsCopy(this->address, msource.GetAddress(), preall_rows*preall_columns);
						}
									/// Destructor
	virtual inline ~ChMatrixNM () { 
						};

			//
			// OPERATORS
			//

					/// Assignment operator (from generic other matrix, acceptable only if other matrix has same size)
	ChMatrixNM<Real,preall_rows,preall_columns>& operator =(const ChMatrix<Real>& matbis)
						{
							assert(matbis.GetColumns()==preall_columns && matbis.GetRows()==preall_rows);
							ChMatrix<Real>::operator=(matbis);
							return *this;
						}

					///	Negates sign of the matrix. 
					/// Performance warning: a new object is created.
	ChMatrixNM<Real,preall_rows,preall_columns> operator-() 
						{ 
							ChMatrixNM<Real,preall_rows,preall_columns> result (*this); 
							result.MatrNeg(); 
							return result;
						};

					///	Sums this matrix and another matrix (of same size) 
					/// Performance warning: a new object is created.
	template <class RealB>
	ChMatrixNM<Real,preall_rows,preall_columns> operator+(const ChMatrix<RealB>& matbis) 
						{ 
							ChMatrixNM<Real,preall_rows,preall_columns> result; 
							result.MatrAdd (*this, matbis); 
							return result;
						};

					///	Subtracts this matrix and another matrix (of same size). 
					/// Performance warning: a new object is created.
	template <class RealB>
    ChMatrixNM<Real,preall_rows,preall_columns> operator-(const ChMatrix<RealB>& matbis) 
						{ 
							ChMatrixNM<Real,preall_rows,preall_columns> result; 
							result.MatrSub (*this, matbis); 
							return result;
						};

					///	Multiplies this matrix and another ChMatrixNM matrix.
					/// This is optimized: it returns another ChMatrixMN because size of matbis is known statically.
					/// Performance warning: a new object is created.
	template <class RealB, int B_rows, int B_columns>
	ChMatrixNM<Real,preall_rows,B_columns> operator*(const ChMatrixNM<RealB,preall_columns,B_columns>& matbis)	
						{ 
							ChMatrixNM<Real, preall_rows,B_columns> result;
							result.MatrMultiply (*this, matbis); 
							return result;
						}

					///	Multiplies this matrix and another generic matrix.
					/// Returns a ChMatrixDynamic matrix, because size of matbis is not known at compile time.
					/// Performance warning: a new object is created.
	template <class RealB>
	ChMatrixDynamic<Real> operator*(const ChMatrix<RealB>& matbis)	
						{ 
							ChMatrixDynamic<Real> result (this->rows, matbis.GetColumns());
							result.MatrMultiply (*this, matbis); 
							return result;
						};

					///	Multiplies this matrix by a scalar value
					/// Performance warning: a new object is created.
	ChMatrixNM<Real,preall_rows,preall_columns> operator*(const Real factor) 
						{ ChMatrixNM<Real,preall_rows,preall_columns> result (*this); result.MatrScale (factor); return result;};



			//
			// FUNCTIONS
			//
									/// Resize for this matrix is NOT SUPPORTED ! DO NOTHING!
	virtual inline void Resize(int nrows, int ncols)
						{
							assert ((nrows == this->rows) && (ncols == this->columns));
						}

};






///
/// ChMatrix33
///
/// A special type of NxM matrix: the 3x3 matrix that is commonly used 
/// to represent coordinate transformations in 3D space.
/// This matrix cannot be resized.
/// The 3x3 matrix can be multiplied/added with other matrix types.
///


template <class Real = double>
class ChMatrix33 : public ChMatrixNM<Real,3,3>
{
public:
			//
			// CONSTRUCTORS
			//

					/// Default constructor builds a 3x3 matrix with zeroes.
	inline ChMatrix33() : ChMatrixNM<Real,3,3>() {};

					/// Copy constructor 
	inline ChMatrix33 (const ChMatrix33<Real>& msource) : ChMatrixNM<Real,3,3>(msource) {};

					/// Copy constructor from all types of base matrices (only with same size)
	template <class RealB>
	inline ChMatrix33 (const ChMatrix<RealB>& msource) 
						{
							assert(msource.GetColumns()==3 && msource.GetRows()==3);
							this->rows= 3;
							this->columns = 3;
							this->address = this->buffer;
							ElementsCopy(this->address, msource.GetAddress(), 9); 
						}

					/// The constructor which builds a 3x3 matrix given a 
					/// quaternion representing rotation
	template <class RealB>
	inline ChMatrix33 (const ChQuaternion<RealB>& mq)   
						{ 
							this->rows= 3;
							this->columns = 3;
							this->address = this->buffer;
							Set_A_quaternion(mq);
						}


			//
			// OPERATORS
			//
						/// Assignment operator (from generic other matrix, acceptable only if other matrix has 3x3 size)
	inline ChMatrix33<Real>&  operator =(const ChMatrix<Real>& matbis)
						{
							assert(matbis.GetColumns()==3 && matbis.GetRows()==3);
							ChMatrix<Real>::operator=(matbis);
							return *this;
						}

					///	Negates sign of the matrix. 
					/// Performance warning: a new object is created.
	ChMatrix33<Real> operator-() 
						{ 
							ChMatrix33<Real> result (*this); 
							result.MatrNeg(); 
							return result;
						};

					///	Sums this matrix and another matrix (of same size)
					/// Performance warning: a new object is created.
	template <class RealB>
	ChMatrix33<Real> operator+(const ChMatrix<RealB>& matbis) 
						{ 
							ChMatrix33<Real> result; 
							result.MatrAdd (*this, matbis); 
							return result;
						};

					///	Subtracts this matrix and another matrix (of same size). 
					/// Performance warning: a new object is created.
	template <class RealB>
    ChMatrix33<Real> operator-(const ChMatrix<RealB>& matbis) 
						{ 
							ChMatrix33<Real> result; 
							result.MatrSub (*this, matbis); 
							return result;
						};

					///	Multiplies this ChMatrix33 matrix and another ChMatrix33 matrix. 
					/// Performance warning: a new object is created.
	template <class RealB>
	ChMatrix33<Real> operator*(const ChMatrix33<RealB>& matbis)	
						{ 
							ChMatrix33<Real> result; 
							result.MatrMultiply (*this, matbis); 
							return result;
						}

					///	Multiplies this matrix and another ChMatrixNM matrix (3xN). 
					/// Performance warning: a new object is created (of ChMatrixNM type).
	template <class RealB, int B_rows, int B_columns>
	ChMatrixNM<Real,3, B_columns> operator*(const ChMatrixNM<RealB,3, B_columns>& matbis)	
						{ 
							ChMatrixNM<Real, 3, B_columns> result; // try statical sizing
							result.MatrMultiply (*this, matbis); 
							return result;
						}

					///	Multiplies this matrix and another generic matrix. 
					/// Performance warning: a new object is created (of ChMatrixDynamic type).
	template <class RealB>
	ChMatrixDynamic<Real> operator*(const ChMatrix<RealB>& matbis)	
						{ 
							ChMatrixDynamic<Real> result (this->rows, matbis.GetColumns()); 
							result.MatrMultiply (*this, matbis); 
							return result;
						};

					///	Multiplies this matrix by a scalar value
					/// Performance warning: a new object is created.
	ChMatrix33<Real> operator*(const Real factor) 
						{ ChMatrix33<Real> result (*this); result.MatrScale (factor); return result;};

					///	Multiplies this matrix by a vector.
	ChVector<Real> operator*(const ChVector<Real>& myvect) 
						{ return (this->Matr_x_Vect (myvect)); };

			//
			// FUNCTIONS
			//

					/// Reset to identity a 3x3 matrix (ones on diagonal, zero elsewhere)
					/// Note: optimized, for 3x3 matrices ONLY!
	void Set33Identity()
						{
							this->Reset();
							this->Set33Element(0,0, 1.0); this->Set33Element(1,1, 1.0); this->Set33Element(2,2, 1.0);
						}	

					/// Multiplies this matrix by a vector, like in coordinate rotation [M]*v. 
					///  \return The result of the multiplication, i.e. a vector.
	template <class RealB>
	inline ChVector<Real> Matr_x_Vect	(ChVector<RealB> va) const
						{
							return ChVector<Real> (
										(( this->Get33Element(0,0))*va.x)+
										(( this->Get33Element(0,1))*va.y)+
										(( this->Get33Element(0,2))*va.z),
										(( this->Get33Element(1,0))*va.x)+
										(( this->Get33Element(1,1))*va.y)+
										(( this->Get33Element(1,2))*va.z),
										(( this->Get33Element(2,0))*va.x)+
										(( this->Get33Element(2,1))*va.y)+
										(( this->Get33Element(2,2))*va.z) );
						}

					/// Multiplies this matrix (transposed) by a vector, as [M]'*v
					///  \return The result of the multiplication, i.e. a vector.
	template <class RealB>
	inline ChVector<Real> MatrT_x_Vect	(ChVector<RealB> va) const
						{
							return ChVector<Real> (
										((this->Get33Element(0,0))*va.x)+
										((this->Get33Element(1,0))*va.y)+
										((this->Get33Element(2,0))*va.z)  ,
										((this->Get33Element(0,1))*va.x)+
										((this->Get33Element(1,1))*va.y)+
										((this->Get33Element(2,1))*va.z)  ,
										((this->Get33Element(0,2))*va.x)+
										((this->Get33Element(1,2))*va.y)+
										((this->Get33Element(2,2))*va.z)  );
						}

					/// Fast inversion of small matrices. Result will be in 'matra'.
					/// \return Returns the determinant.
	template <class RealB>
	double FastInvert (ChMatrix33<RealB>* matra)
					{
						double det;
						double sdet0, sdet1, sdet2;

						sdet0 =+(this->Get33Element(1,1) * this->Get33Element(2,2))
							   -(this->Get33Element(2,1) * this->Get33Element(1,2));
						sdet1 =-(this->Get33Element(1,0) * this->Get33Element(2,2))
							   +(this->Get33Element(2,0) * this->Get33Element(1,2));
						sdet2 =+(this->Get33Element(1,0) * this->Get33Element(2,1))
							   -(this->Get33Element(2,0) * this->Get33Element(1,1));

						det = sdet0 * this->Get33Element(0,0) +
							  sdet1 * this->Get33Element(0,1) +
							  sdet2 * this->Get33Element(0,2);

						matra->Set33Element (0,0, sdet0/det);
						matra->Set33Element (1,0, sdet1/det);
						matra->Set33Element (2,0, sdet2/det);
						matra->Set33Element (0,1,(-(this->Get33Element(0,1) * this->Get33Element(2,2))
												  +(this->Get33Element(2,1) * this->Get33Element(0,2)))/det);
						matra->Set33Element (1,1,(+(this->Get33Element(0,0) * this->Get33Element(2,2))
												  -(this->Get33Element(2,0) * this->Get33Element(0,2)))/det);
						matra->Set33Element (2,1,(-(this->Get33Element(0,0) * this->Get33Element(2,1))
												  +(this->Get33Element(2,0) * this->Get33Element(0,1)))/det);
						matra->Set33Element (0,2,(+(this->Get33Element(0,1) * this->Get33Element(1,2))
												  -(this->Get33Element(1,1) * this->Get33Element(0,2)))/det);
						matra->Set33Element (1,2,(-(this->Get33Element(0,0) * this->Get33Element(1,2))
												  +(this->Get33Element(1,0) * this->Get33Element(0,2)))/det);
						matra->Set33Element (2,2,(+(this->Get33Element(0,0) * this->Get33Element(1,1))
												  -(this->Get33Element(1,0) * this->Get33Element(0,1)))/det);

						return (det);
					}


	#define EIG_ROTATE(a,i,j,k,l) g=a->Get33Element(i,j); h=a->Get33Element(k,l); a->Set33Element(i,j,g-s*(h+g*tau)); a->Set33Element(k,l,h+s*(g-h*tau));

					/// Returns 3 eigenvalues and 3 eigenvectors in a 3x3 matrix, 
					/// Notes:
					///   - only for cases where eigenvalues are real!! 
					///   - output eigenvectors as columns of matrix 'eigenvects'
					///     (which will be resized to 3x3 if not yet so..)
					///   - this original matrix is modified
	void FastEigen(ChMatrix33<Real>& eigenvects, double eigenvals[])
					{
						  int n = 3;
						  int j,iq,ip,i;
						  double tresh,theta,tau,t,sm,s,h,g,c;
						  int nrot;
						  double b[3];
						  double z[3];
						  double d[3];
						  ChMatrix33<double> v;
						  //ChMatrix33<double> vout;

						  v.Set33Identity();

						  for(ip=0; ip<n; ip++) 
							{
							  b[ip] = this->Get33Element(ip,ip);
							  d[ip] = this->Get33Element(ip,ip);
							  z[ip] = 0.0;
							}
						  
						  nrot = 0;
						  
						  for(i=0; i<50; i++)
						  {

							sm=0.0;
							for(ip=0;ip<n;ip++)
								for(iq=ip+1;iq<n;iq++) 
									sm+=fabs(this->Get33Element(ip,iq));
						    
							if (sm == 0.0)
							{
							  //vout.CopyFromMatrix(v);
							  //dout[0] = d[0];
							  //dout[1] = d[1];
							  //dout[2] = d[2];
							  return;
							}    
						      
							if (i < 3) tresh=(double)0.2*sm/(n*n);
							  else tresh=0.0;
						      
							for(ip=0; ip<n; ip++) for(iq=ip+1; iq<n; iq++)
							{
							  g = (double)100.0*fabs(this->Get33Element(ip,iq));
							  if (i>3 && 
								  fabs(d[ip])+g==fabs(d[ip]) && 
								  fabs(d[iq])+g==fabs(d[iq]))
								this->Set33Element(ip,iq,0.0);
							  else if (fabs(this->Get33Element(ip,iq))>tresh)
								{
								  h = d[iq]-d[ip];
								  if (fabs(h)+g == fabs(h)) t=(this->Get33Element(ip,iq))/h;
								  else
								{
								  theta=(double)0.5*h/(this->Get33Element(ip,iq));
								  t=(double)(1.0/(fabs(theta)+sqrt(1.0+theta*theta)));
								  if (theta < 0.0) t = -t;
								}
								  c=(double)1.0/sqrt(1+t*t);
								  s=t*c;
								  tau=s/((double)1.0+c);
								  h=t*this->Get33Element(ip,iq);
								  z[ip] -= h;
								  z[iq] += h;
								  d[ip] -= h;
								  d[iq] += h;
								  this->Set33Element(ip,iq,0.0);
								  for(j=0;j<ip;j++) { EIG_ROTATE(this,j,ip,j,iq); } 
								  for(j=ip+1;j<iq;j++) { EIG_ROTATE(this,ip,j,j,iq); } 
								  for(j=iq+1;j<n;j++) { EIG_ROTATE(this,ip,j,iq,j); } 
								  for(j=0;j<n;j++) { EIG_ROTATE((&v),j,ip,j,iq); } 
								  nrot++;
								}
							}

							for(ip=0;ip<n;ip++)
							{
							  b[ip] += z[ip];
							  d[ip] = b[ip];
							  z[ip] = 0.0;
							}
						  }

						  return;
					}

					/// Fills a 3x3 matrix as a rotation matrix corresponding
					/// to the rotation expressed by the quaternion 'quat'.
	template <class RealB>
	void Set_A_quaternion( ChQuaternion<RealB> quat )
					{
						double e0e0;
						double e1e1;
						double e2e2;
						double e3e3;
						double e0e1;
						double e0e2;
						double e0e3;
						double e1e2;
						double e1e3;
						double e2e3;

						e0e0 = quat.e0 * quat.e0;
						e1e1 = quat.e1 * quat.e1;
						e2e2 = quat.e2 * quat.e2;
						e3e3 = quat.e3 * quat.e3;
						e0e1 = quat.e0 * quat.e1;
						e0e2 = quat.e0 * quat.e2;
						e0e3 = quat.e0 * quat.e3;
						e1e2 = quat.e1 * quat.e2; 
						e1e3 = quat.e1 * quat.e3; 
						e2e3 = quat.e2 * quat.e3; 
					 
						this->Set33Element (0,0, (e0e0 + e1e1) * 2 - 1);
						this->Set33Element (0,1, (e1e2 - e0e3) * 2 );
						this->Set33Element (0,2, (e1e3 + e0e2) * 2 );
						this->Set33Element (1,0, (e1e2 + e0e3) * 2 );
						this->Set33Element (1,1, (e0e0 + e2e2) * 2 - 1);
						this->Set33Element (1,2, (e2e3 - e0e1) * 2 );
						this->Set33Element (2,0, (e1e3 - e0e2) * 2 );
						this->Set33Element (2,1, (e2e3 + e0e1) * 2 );
						this->Set33Element (2,2, (e0e0 + e3e3) * 2 - 1);
					}
					
					/// Fills a 3x3 matrix as the "star" matrix, representing vector cross product.
					/// That is, given two 3d vectors a and b, aXb= [Astar]*b
	template <class RealB>
	void Set_X_matrix	( ChVector<RealB> vect )
					{
						this->Set33Element (0,0,0);
						this->Set33Element (0,1,(- vect.z));
						this->Set33Element (0,2,(  vect.y));
						this->Set33Element (1,0,(  vect.z));
						this->Set33Element (1,1,0);
						this->Set33Element (1,2,(- vect.x));
						this->Set33Element (2,0,(- vect.y));
						this->Set33Element (2,1,(  vect.x));
						this->Set33Element (2,2,0);
					}

						/// Fills a 3x3 matrix as a rotation matrix, given the three 
					/// versors X,Y,Z of the basis.
	template <class RealB>
	void Set_A_axis		 ( ChVector<RealB> X, ChVector<RealB> Y, ChVector<RealB> Z)
					{
						this->Set33Element (0,0,X.x);
						this->Set33Element (0,1,Y.x);
						this->Set33Element (0,2,Z.x);
						this->Set33Element (1,0,X.y);
						this->Set33Element (1,1,Y.y);
						this->Set33Element (1,2,Z.y);
						this->Set33Element (2,0,X.z);
						this->Set33Element (2,1,Y.z);
						this->Set33Element (2,2,Z.z);
					}

					/// Fills a 3x3 matrix as a rotation matrix, given the three
					/// Eulero angles (not to be confused with 'Eulero parameters', aka quaternions) 
	template <class RealB>
	void Set_A_Eulero	 ( ChVector<RealB> eul )
					{
						double cx, cy, cz, sx, sy, sz;
						cx= cos(eul.x);
						cy= cos(eul.y);
						cz= cos(eul.z);
						sx= sin(eul.x);
						sy= sin(eul.y);
						sz= sin(eul.z);

						this->Set33Element (0,0,((cz*cx)-(cy*sx*sz)));
						this->Set33Element (0,1,(-(sz*cx)-(cy*sx*cz)));
						this->Set33Element (0,2,(sy*sx));
						this->Set33Element (1,0,((cz*sx)+(cy*cx*sz)));
						this->Set33Element (1,1,(-(sz*sx)+(cy*cx*cz)));
						this->Set33Element (1,2,(-sy*cx));
						this->Set33Element (2,0,(sy*sz));
						this->Set33Element (2,1,(sy*cz));
						this->Set33Element (2,2,(cy));
					}

					/// Fills a 3x3 matrix as a rotation matrix, given the three
					/// Cardano angles.
	template <class RealB>
	void Set_A_Cardano	 ( ChVector<RealB> car )
					{
						double cx, cy, cz, sx, sy, sz;
						cx= cos(car.x);
						cy= cos(car.y);
						cz= cos(car.z);
						sx= sin(car.x);
						sy= sin(car.y);
						sz= sin(car.z);

						this->Set33Element (0,0,((cx*cz)-(sz*sx*sy)));
						this->Set33Element (0,1,(-sx*cy));
						this->Set33Element (0,2,((cx*sz)+(sx*sy*cz)));
						this->Set33Element (1,0,((sx*cz)+(cx*sy*sz)));
						this->Set33Element (1,1,(cy*cx));
						this->Set33Element (1,2,((sx*sz)-(cx*sy*cz)));
						this->Set33Element (2,0,(-sz*cy));
						this->Set33Element (2,1,(sy));
						this->Set33Element (2,2,(cy*cz));
					}

					/// Fills a 3x3 matrix as a rotation matrix, given the three
					/// head, pitch, banking  angles.
	template <class RealB>
	void Set_A_Hpb		 ( ChVector<RealB> hpb )
					{
						double cx, cy, cz, sx, sy, sz;
						cx= cos(hpb.y);
						cy= cos(hpb.x);
						cz= cos(hpb.z);
						sx= sin(hpb.y);
						sy= sin(hpb.x);
						sz= sin(hpb.z);

						this->Set33Element (0,0,((cz*cy)-(sz*sx*sy)));
						this->Set33Element (0,1,(-(sz*cy)-(cz*sx*sy)));
						this->Set33Element (0,2,(-cx*sy));
						this->Set33Element (1,0,(sz*cx));
						this->Set33Element (1,1,(cz*cx));
						this->Set33Element (1,2,(-sx));
						this->Set33Element (2,0,((cz*sy)+(sz*sx*cy)));
						this->Set33Element (2,1,(-(sz*sy)+(cz*sx*cy)));
						this->Set33Element (2,2,(cx*cy));
					}

					/// Fills a 3x3 matrix as a rotation matrix, given the three
					/// angles of consecutive rotations about x,y,z axis.
	template <class RealB>
	void Set_A_Rxyz		 ( ChVector<RealB> xyz )
					{
						double cx, cy, cz, sx, sy, sz;
						cx= cos(xyz.x);
						cy= cos(xyz.y);
						cz= cos(xyz.z);
						sx= sin(xyz.x);
						sy= sin(xyz.y);
						sz= sin(xyz.z);

						this->Set33Element (0,0,(cy*cz));
						this->Set33Element (0,1,(cy*sz));
						this->Set33Element (0,2,(-sy));
						this->Set33Element (1,0,((sx*sy*cz)-(cx*sz)));
						this->Set33Element (1,1,((sx*sy*sz)+(cx*cz)));
						this->Set33Element (1,2,(sx*cy));
						this->Set33Element (2,0,((cx*sy*cz)+(sx*sz)));
						this->Set33Element (2,1,((cx*sy*sz)-(sx*cz)));
						this->Set33Element (2,2,(cx*cy));
					}


					/// Fills a 3x3 matrix as a rotation matrix, given the three
					/// Rodriguez' parameters.
	template <class RealB>
	void Set_A_Rodriguez ( ChVector<RealB> rod )
					{
						double gam= pow(rod.x,2)+pow(rod.y,2)+pow(rod.z,2);

						this->Set33Element (0,0,(1+pow(rod.x,2)-pow(rod.y,2)-pow(rod.z,2)));
						this->Set33Element (0,1,(2*(rod.x*rod.y - rod.z)));
						this->Set33Element (0,2,(2*(rod.x*rod.z + rod.y)));
						this->Set33Element (1,0,(2*(rod.x*rod.y + rod.z)));
						this->Set33Element (1,1,(1-pow(rod.x,2)+pow(rod.y,2)-pow(rod.z,2)));
						this->Set33Element (1,2,(2*(rod.y*rod.z - rod.x)));
						this->Set33Element (2,0,(2*(rod.x*rod.z - rod.y)));
						this->Set33Element (2,1,(2*(rod.y*rod.z + rod.x)));
						this->Set33Element (2,2,(1-pow(rod.x,2)-pow(rod.y,2)+pow(rod.z,2)));

						this->MatrScale((1/(1+gam)));
					}


					/// Given a 3x3 rotation matrix, computes the corresponding 
					/// quaternion.
	ChQuaternion<Real> Get_A_quaternion ()
					{
						ChQuaternion<Real> q;
						double s, tr;
													// for speed reasons: ..
						double m00 = this->Get33Element(0,0);
						double m01 = this->Get33Element(0,1);
						double m02 = this->Get33Element(0,2);
						double m10 = this->Get33Element(1,0);
						double m11 = this->Get33Element(1,1);
						double m12 = this->Get33Element(1,2);
						double m20 = this->Get33Element(2,0);
						double m21 = this->Get33Element(2,1);
						double m22 = this->Get33Element(2,2);

						tr=m00 + m11 + m22;		// diag sum

						if (tr >= 0)
						{
							s = sqrt(tr + 1);
							q.e0 = 0.5 * s;
							s = 0.5 / s;
							q.e1 = (m21 - m12) * s;
							q.e2 = (m02 - m20) * s;
							q.e3 = (m10 - m01) * s;
						}
						else
						{
							int i = 0;

							if (m11 > m00)
							{	
								i = 1;
								if (m22 > m11)	i = 2;
							}
							else
							{
								if (m22 > m00)  i = 2;
							}

							switch (i)
							{
							case 0:
								s = sqrt (m00 - m11 - m22 + 1);
								q.e1 = 0.5 * s;
								s = 0.5 / s;
								q.e2 = (m01 + m10) * s;
								q.e3 = (m20 + m02) * s;
								q.e0 = (m21 - m12) * s;
								break;
							case 1:
								s = sqrt (m11 - m22 - m00 + 1);
								q.e2 = 0.5 * s;
								s = 0.5 / s;
								q.e3 = (m12 + m21) * s;
								q.e1 = (m01 + m10) * s;
								q.e0 = (m02 - m20) * s;
								break;
							case 2:
								s = sqrt (m22 - m00 - m11 + 1);
								q.e3 = 0.5 * s;
								s = 0.5 / s;
								q.e1 = (m20 + m02) * s;
								q.e2 = (m12 + m21) * s;
								q.e0 = (m10 - m01) * s;
								break;
							}
						}
						return q;
					}

					/// Given a 3x3 rotation matrix, returns the versor of X axis.
	ChVector<Real> Get_A_Xaxis ()
					{
						ChVector<Real> X;
						X.x= this->Get33Element (0,0);
						X.y= this->Get33Element (1,0);
						X.z= this->Get33Element (2,0);
						return X;
					}

					/// Given a 3x3 rotation matrix, returns the versor of Y axis.
	ChVector<Real> Get_A_Yaxis ()
					{
						ChVector<Real> Y;
						Y.x= this->Get33Element (0,1);
						Y.y= this->Get33Element (1,1);
						Y.z= this->Get33Element (2,1);
						return Y;
					}

					/// Given a 3x3 rotation matrix, returns the versor of Z axis.
	ChVector<Real> Get_A_Zaxis ()
					{
						ChVector<Real> Z;
						Z.x= this->Get33Element (0,2);
						Z.y= this->Get33Element (1,2);
						Z.z= this->Get33Element (2,2);
						return Z;
					}

					/// Given a 3x3 rotation matrix, returns the Eulero angles.
	ChVector<Real> Get_A_Eulero ()
					{
						ChVector<Real> eul;

						eul.y= acos(this->GetElement(2,2));	// rho, nutation

						eul.z= acos((this->GetElement(2,1))/sin(eul.y));	// csi, spin

						eul.x= acos(-(this->GetElement(1,2))/sin(eul.y));	// rho, nutation

						if (eul.y == 0)
						{		// handle undefined initial position set
							eul.x = 0;
							eul.z = 0;
						}

						return (eul);
					}

					/// Given a 3x3 rotation matrix, returns the Cardano angles.
	ChVector<Real> Get_A_Cardano ()
					{
						ChVector<Real> car;

						double mel21 = this->GetElement(2,1);
						if (mel21 >1)  mel21 =  1;
						if (mel21 <-1) mel21 = -1;

						car.y= asin(mel21);

						double arg2 = (this->GetElement(2,2))/cos(car.y);
						if (arg2 >1)  arg2 =  1;
						if (arg2 <-1) arg2 = -1;
						double arg3 = (this->GetElement(1,1))/cos(car.y);
						if (arg3 >1)  arg3 =  1;
						if (arg3 <-1) arg3 = -1;

						car.z= acos(arg2);
						car.x= acos(arg3);

						return (car);
					}

					/// Given a 3x3 rotation matrix, returns the head-pitch-banking angles.
	ChVector<Real> Get_A_Hpb ()
					{
						ChVector<Real> Hpb;

						double arg1 = -(this->GetElement(1,2));
						if (arg1 >1)  arg1 =  1;
						if (arg1 <-1) arg1 = -1;

						Hpb.y= asin(arg1);	// P

						double arg2 = (this->GetElement(2,2))/cos(Hpb.y);
						if (arg2 >1)  arg2 =  1;
						if (arg2 <-1) arg2 = -1;
						double arg3 = (this->GetElement(1,1))/cos(Hpb.y);
						if (arg3 >1)  arg3 =  1;
						if (arg3 <-1) arg3 = -1;

						Hpb.x= acos(arg2); // H
						Hpb.z= acos(arg3); // B

						return (Hpb);
					}

					/// Given a 3x3 rotation matrix, returns the angles for
					/// consecutive rotations on x,y,z axes..
	ChVector<Real> Get_A_Rxyz ()
					{
						ChVector<Real> Rxyz;

						double arg1 = -(this->GetElement(0,2));
						if (arg1 >1)  arg1 =  1;
						if (arg1 <-1) arg1 = -1;
						Rxyz.y= asin(arg1);

						double arg2 = (this->GetElement(0,1))/cos(Rxyz.y);
						if (arg2 >1)  arg2 =  1;
						if (arg2 <-1) arg2 = -1;
						double arg3 = (this->GetElement(1,2))/cos(Rxyz.y);
						if (arg3 >1)  arg3 =  1;
						if (arg3 <-1) arg3 = -1;

						Rxyz.z= asin(arg2);
						Rxyz.x= asin(arg3);

						return (Rxyz);
					}

					/// Given a 3x3 rotation matrix, returns the Rodriguez parameters.
	ChVector<Real> Get_A_Rodriguez ()
					{
						ChVector<Real> rod;
						ChQuaternion<Real> qtemp;
						qtemp = Get_A_quaternion();
						// warning: infinite results may happen..
						rod.x = qtemp.e1 / qtemp.e0;
						rod.y = qtemp.e2 / qtemp.e0;
						rod.z = qtemp.e3 / qtemp.e0;

						return (rod);
					}

};






//
// Conversion from/to matrix declarated as double[3][3]  \todo implement as class members
//
ChApi
void Chrono_to_Marray   (ChMatrix33<>& ma, double marr[3][3]);

ChApi
void Chrono_from_Marray (ChMatrix33<>& ma, double marr[3][3]);









} // END_OF_NAMESPACE____



#endif  // END of ChMatrix.h 
