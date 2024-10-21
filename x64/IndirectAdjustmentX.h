// IndirectAdjustmentX.h: interface for the CIndirectAdjustmentX class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_INDIRECTADJUSTMENTX_H__4A7BD844_9392_49DF_9428_BD6C72DFA1E4__INCLUDED_)
#define AFX_INDIRECTADJUSTMENTX_H__4A7BD844_9392_49DF_9428_BD6C72DFA1E4__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

inline void SubLdltban1(double* a, double* d, double* l, int n, int wide) {
	int i, j, k, kk, km, m;
	double* ao, * aa, * co, * c;

	m = wide * (2 * n + 1 - wide) / 2;
	c = (double*)calloc((m - wide), sizeof(double));

	ao = a; co = c; a += wide;
	for (i = 0; i < m - wide; i++) *c++ = *a++;
	c = co; a = ao;

	kk = 0;

	for (k = 1; k < n; k++) {
		if (k < n - wide + 2) kk = wide - 1;
		else kk--;

		*d = *a++; aa = a;  a += kk;

		if (k < n - wide + 1) km = wide;
		else km = n - k + 1;

		for (i = 1; i < kk + 1; i++) {
			*l = *aa++ / *d;
			for (j = 0; j < kk - i + 1; j++) *(a + j) -= *l * *(aa + j - 1);
			l++;

			if (k + i > n - wide + 1) km--;
			a += km;
		}

		a = aa; d++;
		if (k == n - 1)  *d = *a;
	}

	a = ao;  a += wide;
	for (i = 0; i < m - wide; i++) *a++ = *c++;
	c = co; free(c);
};

inline void SubLdltban2(double* l, double* d, double* b, double* x, int n, int wide) {
	int i, j, kk, m;
	double* bo, * lo, * xx;
	double* bb, * bbo;
	kk = 0;

	bb = (double*)calloc(n, sizeof(double));
	bbo = bb;

	bo = b; lo = l;

	for (i = 0; i < n; i++)*bb++ = *b++;
	b = bo;  bb = bbo;
	m = wide * (2 * n + 1 - wide) / 2;

	for (i = 1; i < n; i++) {
		if (i < n - wide + 2) kk = wide;
		else kk--;

		b = bo + i;
		for (j = 1; j < kk; j++) *b++ -= *(b - j) * *l++;
	}

	kk = 0;
	b = bo + n - 1;  l = lo + m - n - 1;
	x += n - 1;  xx = x;  d += n - 1;

	*x-- = (*b--) / (*d--);

	for (i = 1; i < n; i++) {
		if (i < wide) kk++;
		else { kk = wide - 1;  xx--; }

		*x = *b-- / *d--;
		for (j = 1; j < kk + 1; j++) *x -= *l-- * *(xx - j + 1);
		x--;
	}

	b = bo;
	for (i = 0; i < n; i++) *b++ = *bb++;
	bb = bbo; free(bb);
};

// construct the normalize equations
inline void NormalizeEquation(double* aa, int n, double bb, double* a, double* b) {
	register int  i, j;
	double* a0 = a;

	for (i = 0; i < n; i++) {
		for (j = 0; j < n - i; j++) {
			*a += *aa * *(aa + j);
			a++;
		}
		*b += *aa * bb;
		b++; aa++;
	}

	a = a0;
};

// construct the normalize equations
inline void NormalizeEquation(double* aa, int n, double bb, double* a, double* b, double p) {
	register int  i, j;
	double* a0 = a;

	for (i = 0; i < n; i++) {
		for (j = 0; j < n - i; j++) {
			*a += ((*aa) * (*(aa + j)) * p);
			a++;
		}
		*b += ((*aa) * bb * p);
		b++; aa++;
	}

	a = a0;
};

// solve the normalized equation
inline void SolveNrmlEquation(double* a, double* b, double* x, int n, int wide) {
	int    m;
	double* d, * l;

	m = n * (n + 1) / 2;
	d = (double*)malloc(n * sizeof(double));
	l = (double*)malloc((m - n) * sizeof(double));

	memset(d, 0, n * sizeof(double));
	memset(l, 0, (m - n) * sizeof(double));

	SubLdltban1(a, d, l, n, wide);
	SubLdltban2(l, d, b, x, n, wide);

	free(d);
	free(l);
};

//////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Function ��   cagaus	
//	Description�� Solve the linear equations
//	Input:        a -- the parameters matrix ( n*n )
//                b -- the constant vector on the right hand of the equation ( n*1 )
//                n -- the number of the unknown
//	Output��      x -- the unknown vector
//	InOut:        None
//  Return Value: None
//	Exception:    None 
//
//////////////////////////////////////////////////////////////////////////////////////////////////
inline int CalcLinearEquationByGauss(double* a, double* b, int n, double* x) {
	int* js, l, k, i, j, is, p, q;
	double d, t;

	is = 0;
	js = (int*)malloc(n * sizeof(int));
	memset(js, 0, n * sizeof(int));
	l = 1;

	for (k = 0; k <= n - 2; k++) {
		d = 0.0;
		for (i = k; i <= n - 1; i++) {
			for (j = k; j <= n - 1; j++) {
				t = fabs(a[i * n + j]);
				if (t > d) {
					d = t;
					js[k] = j;
					is = i;
				}
			}
		}

		if (d + 1.0 == 1.0) l = 0;
		else {
			if (js[k] != k) {
				for (i = 0; i <= n - 1; i++) {
					p = i * n + k;
					q = i * n + js[k];
					t = a[p];
					a[p] = a[q];
					a[q] = t;
				}
			}
			if (is != k) {
				for (j = k; j <= n - 1; j++) {
					p = k * n + j;
					q = is * n + j;
					t = a[p];
					a[p] = a[q];
					a[q] = t;
				}
				t = b[k];
				b[k] = b[is];
				b[is] = t;
			}
		}
		if (l == 0) {
			free(js);
			return 0;
		}
		d = a[k * n + k];
		for (j = k + 1; j <= n - 1; j++) {
			p = k * n + j;
			a[p] = a[p] / d;
		}
		b[k] = b[k] / d;
		for (i = k + 1; i <= n - 1; i++) {
			for (j = k + 1; j <= n - 1; j++) {
				p = i * n + j;
				a[p] = a[p] - a[i * n + k] * a[k * n + j];
			}
			b[i] = b[i] - a[i * n + k] * b[k];
		}
	}

	d = a[(n - 1) * n + n - 1];
	if (fabs(d) + 1.0 == 1.0) {
		free(js);
		return 0;
	}

	x[n - 1] = b[n - 1] / d;
	for (i = n - 2; i >= 0; i--) {
		t = 0.0;
		for (j = i + 1; j <= n - 1; j++)
			t = t + a[i * n + j] * x[j];
		x[i] = b[i] - t;
	}

	js[n - 1] = n - 1;
	for (k = n - 1; k >= 0; k--) {
		if (js[k] != k) {
			t = x[k];
			x[k] = x[js[k]];
			x[js[k]] = t;
		}
	}
	free(js);
	return 1;
};

// �Գ��������������
inline long AtxDesgj(double* a, int n) {
	int i, j, k, m;
	double w, g;
	double* b = new double[n];
	memset(b, 0, n * sizeof(double));
	for (k = 0; k <= n - 1; k++) {
		w = a[0];
		if (fabs(w) + 1.0 == 1.0) {
			delete[] b;
			//			printf("fail\n");
			return -2;
		}
		m = n - k - 1;
		for (i = 1; i <= n - 1; i++) {
			g = a[i * n];
			b[i] = g / w;
			if (i <= m) b[i] = -b[i];
			for (j = 1; j <= i; j++) {
				a[(i - 1) * n + j - 1] = a[i * n + j] + g * b[j];
			}
		}
		a[n * n - 1] = 1.0 / w;
		for (i = 1; i <= n - 1; i++) {
			a[(n - 1) * n + i - 1] = b[i];
		}
	}
	for (i = 0; i <= n - 2; i++) {
		for (j = i + 1; j <= n - 1; j++) {
			a[i * n + j] = a[j * n + i];
		}
	}
	if (b) delete[] b;
	return 2;
};

// �Գ����������Ǿ��������
inline long AtxDesgjX(double* a, int n) {
	int i, j, k, l;
	double* b = new double[n * n];
	memset(b, 0, n * n * sizeof(double));
	for (i = k = l = 0; i < n; i++) {
		for (j = 0; j < n; j++, k++) {
			if (j < i) {
				b[k] = b[j * n + i];
			}
			else {
				b[k] = a[l];
				l++;
			}
		}
	}
	int r = AtxDesgj(b, n);
	memcpy(a, b, n * n * sizeof(double));
	if (b) delete[] b;
	return r;
};

// ����at*p*a
// a  -- m*n
// p  -- m
// at -- n*m
// aa -- n*n
inline void NormalizeAtPA(double* a, double* p, int m, int n, double* aa) {
	memset(aa, 0, n * n * sizeof(double));
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			for (int k = 0; k < m; k++) {
				aa[i * n + j] += (p[k] * a[k * n + i] * a[k * n + j]);
			}
		}
	}
};

// ����at*p*b
// a  -- m*n
// p  -- m
// b  -- m*l
// at -- n*m
// ab -- n*l
inline void NormalizeAtPB(double* a, double* p, double* b, int m, int n, int l, double* ab) {
	memset(ab, 0, n * l * sizeof(double));
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < l; j++) {
			for (int k = 0; k < m; k++)
				ab[i * l + j] += (p[k] * a[k * n + i] * b[k * l + j]);
		}
	}
};

// ����c = a * b
// a -- m*n
// b -- n*l
// c -- m*l
inline void MulitiDblMatrix(double* a, double* b, double* c, int m, int n, int l) {
	memset(c, 0, m * l * sizeof(double));
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < l; j++) {
			for (int k = 0; k < n; k++)
				c[i * l + j] += (a[i * n + k] * b[k * l + j]);
		}
	}
};

// ����d = a * b * c
// a -- m*n
// b -- n*l
// t -- m*k  t = a*b
// c -- k*l
// d -- m*l  d = t*c
inline void MulitiTriMatrix(double* a, double* b, double* c, double* d, int m, int n, int k, int l) {
	double* t = new double[m * k];
	MulitiDblMatrix(a, b, t, m, n, k);
	MulitiDblMatrix(t, c, d, m, k, l);
	if (t) delete[] t;
};

// ������: CIndirectAdjustmentX (2.0)
// ��;:   ���ƽ����
// ע��:   ֧�ָ��������ļ��ƽ��
// ����:   ���� (2004-10-12)

class CAtxIndirectAdjustmentX
{
public:
	CAtxIndirectAdjustmentX() {
		m_nUnknown = 0;
		m_nCondition = 0;
		_nbb = NULL;
		_wx = NULL;
		_w = NULL;
		_c = NULL;
	};
	virtual ~CAtxIndirectAdjustmentX() { ReleaseAll(); };

	inline void SetAdjustment(int nUnknown, int nCondition) {
		m_nUnknown = nUnknown;
		m_nCondition = nCondition;

		_nbb = new double[nUnknown * nUnknown];
		_c = new double[nCondition * nUnknown];
		_w = new double[nUnknown];
		_wx = new double[nCondition];

		memset(_nbb, 0, nUnknown * nUnknown * sizeof(double));
		memset(_c, 0, nCondition * nUnknown * sizeof(double));
		memset(_w, 0, nUnknown * sizeof(double));
		memset(_wx, 0, nCondition * sizeof(double));
	};

	inline void AddObservation(double* a, double l, double p = 1.0) {
		NormalizeEquation(a, m_nUnknown, l, _nbb, _w, p);
	}
	inline void AddCondition(double* c, double wx, int k) {
		memcpy(_c + k * m_nUnknown, c, m_nUnknown * sizeof(double));
		_wx[k] = wx;
	};

	inline void SolveEquation(double* x, bool bCondition = true) {
		memset(x, 0, m_nUnknown * sizeof(double));
		if (m_nCondition < 1) bCondition = false;
		if (bCondition) {
			int n = m_nUnknown + m_nCondition;
			double* A = new double[n * n];
			double* B = new double[n];
			double* C = new double[n];
			memset(A, 0, n * n * sizeof(double));
			memset(B, 0, n * sizeof(double));
			memset(C, 0, n * sizeof(double));
			Normalize(A, B);
			SolveNrmlEquation(A, B, C, n, n);
			memcpy(x, C, m_nUnknown * sizeof(double));
			if (A) delete[] A;
			if (B) delete[] B;
			if (C) delete[] C;
		}
		else {
			double* A = new double[m_nUnknown * m_nUnknown];
			double* B = new double[m_nUnknown];
			memcpy(A, _nbb, m_nUnknown * m_nUnknown * sizeof(double));
			memcpy(B, _w, m_nUnknown * sizeof(double));
			SolveNrmlEquation(A, B, x, m_nUnknown, m_nUnknown);
			if (A) delete[] A;
			if (B) delete[] B;
		}
	};

	// ��λ,׼��������һ��ƽ��
	inline void Reset() {
		memset(_nbb, 0, m_nUnknown * m_nUnknown * sizeof(double));
		memset(_w, 0, m_nUnknown * sizeof(double));
		if (m_nCondition > 0) {
			memset(_wx, 0, m_nCondition * sizeof(double));
			memset(_c, 0, m_nCondition * m_nUnknown * sizeof(double));
		}
	};

	// ����δ֪������󷽲�
	inline void PostVariance(double* s, bool bCondition = true) {
		memset(s, 0, m_nUnknown * sizeof(double));
		if (m_nCondition < 1) bCondition = false;
		if (bCondition) {
			int n = m_nUnknown + m_nCondition;
			double* A = new double[n * n];
			double* B = new double[n];
			memset(A, 0, n * n * sizeof(double));
			memset(B, 0, n * sizeof(double));
			Normalize(A, B);
			AtxDesgjX(A, n);
			for (int i = 0; i < n; i++)
				s[i] = A[i * n + i];
			if (A) delete[] A;
			if (B) delete[] B;
		}
		else {
			double* A = new double[m_nUnknown * m_nUnknown];
			double* B = new double[m_nUnknown];
			memcpy(A, _nbb, m_nUnknown * m_nUnknown * sizeof(double));
			memcpy(B, _w, m_nUnknown * sizeof(double));
			AtxDesgjX(A, m_nUnknown);
			for (int i = 0; i < m_nUnknown; i++)
				s[i] = A[i * m_nUnknown + i];
			if (A) delete[] A;
			if (B) delete[] B;
		}
	};

	// ��÷�����ϵ�����������
	// �����ڼ������۲����, Ȼ����ͳ�Ƽ����ѡȨ����
	// [in] N: ��ά����, m_nUnknown*m_nUnknown
	inline void InverseNrml(double* N, bool bCondition = true) {
		memset(N, 0, m_nUnknown * m_nUnknown * sizeof(double));
		if (m_nCondition < 1) bCondition = false;
		if (bCondition) {
			// ����Ǹ��������ļ��ƽ��
			int n = m_nUnknown + m_nCondition;
			double* A = new double[n * n];
			double* B = new double[n];
			memset(A, 0, n * n * sizeof(double));
			memset(B, 0, n * sizeof(double));
			Normalize(A, B);
			AtxDesgjX(A, n);
			memcpy(N, A, m_nUnknown * m_nUnknown * sizeof(double));
			if (A) delete[] A;
			if (B) delete[] B;
		}
		else {
			// ����Ǽ��ƽ��
			double* A = new double[m_nUnknown * m_nUnknown];
			double* B = new double[m_nUnknown];
			memcpy(A, _nbb, m_nUnknown * m_nUnknown * sizeof(double));
			memcpy(B, _w, m_nUnknown * sizeof(double));
			AtxDesgjX(A, m_nUnknown);
			memcpy(N, A, m_nUnknown * m_nUnknown * sizeof(double));
			if (A) delete[] A;
			if (B) delete[] B;
		}
	};

protected:
	int m_nUnknown;
	int m_nCondition;

	double* _nbb;
	double* _wx;
	double* _w;
	double* _c;

	inline void ReleaseAll() {
		if (_nbb) delete[] _nbb;
		if (_c)   delete[] _c;
		if (_w)   delete[] _w;
		if (_wx)  delete[] _wx;
		_nbb = NULL;
		_wx = NULL;
		_w = NULL;
		_c = NULL;
	};

	inline void Normalize(double* a, double* b) {
		int n = m_nUnknown + m_nCondition;
		memcpy(b, _w, m_nUnknown * sizeof(double));
		memcpy(b + m_nUnknown, _wx, m_nCondition * sizeof(double));

		for (int i = 0, k = 0, l = 0; i < m_nUnknown; i++) {
			for (int j = i; j < m_nUnknown; j++, k++, l++) {
				a[l] = _nbb[k];
			}
			for (int t = 0; t < m_nCondition; t++, l++) {
				int j = t + m_nUnknown;
				a[l] = _c[t * m_nUnknown + i];
			}
		}
	};
};

//////////////////////////////////////////////////////////////////////////
//                                                                      // 
// ˵��: ����Baarda����̽�ⷨ (Data Snooping) �޳��ֲ��˵��            //
//                                                                      // 
// (1)   ���ƽ������۲ⷽ��Ϊ V = A*X - L                           //
//       ������Ϊ (AT*P*A)*X = (AT*P*L)                                 //
//       ���о���P�ǹ۲�Ȩ����, P = 1/Qll                               //
//                                                                      //
// (2)   δ֪����Э��������Ϊ Qxx = N = 1/(AT*P*A)                      //
//       ��������Э��������Ϊ Qvv = 1/P - A*Qxx*AT                      //
//                                                                      //
// (3)   ���� Qvv*P = E - P*A*Qxx*AT,  ��� i���Խ���Ԫ�ؼ���           //
//       Ӧ�ĵ� i���۲�Ķ���۲����r[i]. ���ڲ���صĹ۲�ֵ           //
//       ƽ��, ���ڹ۲�Ȩ���� P�ǶԽǾ���, �����������Ķ���           //
//       �۲����                                                       //
//                     0 <= r[i] <= 1                                   //
//       �����й۲�ֵ�Ķ���۲�������ܺ�Ӧ�õ���ƽ��ϵͳ�Ķ�           //
//       ��۲���                                                       //
//                                                                      //
// (4)   �����֪��λȨ�����s0, ��ô��������׼���в�                 //
//           w[i] = fabs(v[i])/(s0*sqrt(Qvv[i, i]))                     //
//                = fabs(v[i])/(s[i]*sqrt(r[i]))                        //
//       ����s[i]�ǵ� i���۲�ֵ�ľ��� s[i] = sqrt(1.0/p[i])*s0          //
//       ��׼���в�w[i]���ӱ�׼��̬�ֲ�                                 //
//           w[i]/H0 ~ N(0, 1)                                          //
//       ͨ������������ˮƽa0 = 0.1%, ��Ӧ���ٽ�ֵΪ Ka = 3.29          //
//       ��: ��� w[i] <= Ka, ��ô��Ϊ�ù۲�ֵΪ�����۲�ֵ;             //
//       ��֮��� w[i]  > Ka, ��ô��Ϊ�ù۲�ֵ���ܴ��ڴֲ�.             //
//       �����жϷ���"����"����ĸ���Ϊa0 = 0.1%                        //
//                                                                      //
//       ����, ����������鹦ЧBp = 80%, ������"��α"����ĸ���         //
//       Ϊ1-Bp, ��ô��Ӧ�ķ����Ļ�����D0 = 4.13.                       //
//       ��Ӧ���ڲ��ɿ�����ֵΪ:   K[i] = D0/sqrt(r[i]).                //
//       ��Ӧ�Ŀɷ��ֲִ������Ϊ: K[i]*s[i]                            //
//                                                                      //
// (5)   ���δ֪��λȨ�����s0,                                        //
//                                                                      // 
//////////////////////////////////////////////////////////////////////////


// ����: qvv_ii
// ����: �������۲����, ������δ֪�����ٵļ��ƽ��
// [in] A: ���۲ⷽ�̵�ϵ������(n*1)
// [in] N: ������ϵ�����������(n*n)
// [in] p: �۲�Ȩ
// [in] n: δ֪������
// [retVal]: ���ظ�����Э��������Qvv����Ӧ�ĶԽ��߷���
inline double CalcQvv(double* A, double* N, double p, int n) {
	// B = A*N = A*Qxx
	// B[i] = sigma(A[k]*N[k, i]);
	int i = 0;
	double* B = new double[n];
	memset(B, 0, n * sizeof(double));
	for (i = 0; i < n; i++) {
		for (int k = 0; k < n; k++)
			B[i] += (A[k] * N[k * n + i]);
	}
	// ���� B*AT, ʵ���Ͼ�������B��A��������
	double r = 0;
	for (i = 0; i < n; i++)
		r += B[i] * A[i];
	// �ͷ���ʱ�ڴ���Դ
	if (B) delete[] B;
	// ���ظ�����Э��������Qvv����Ӧ�ĶԽ��߷���
	return (1.0 / p - r);
};

// ����: redundancy
// ����: �������۲����, ������δ֪�����ٵļ��ƽ��
// [in] A: ���۲ⷽ�̵�ϵ������()
// [in] N: ������ϵ�����������()
// [in] p: �۲�Ȩ
// [in] n: δ֪������
// [retVal]: ���ض�Ӧ�Ķ���۲����
inline double CalcRedundancy(double* A, double* N, double p, int n) {
	return p * CalcQvv(A, N, p, n);
};

// ����: data_snooping_1
// ����: ��֪��λȨ�����ʱ������̽�ⷨ
//       �����趨������ˮƽa0 = 1%, ��Ӧ���ٽ�ֵΪ Ka = 2.56
// [in] s0:  ��λȨ����� 
// [in] v:   �۲�ֵ�Ĳв�
// [in] p:   �۲�ֵ��Ȩ
// [in] r:   �۲�ֵ�Ķ���۲����
// [retVal]: ����дֲ�, ����true, ��֮����false
inline bool DataSnooping(double s0, double v, double p, double r) {
	// �����׼���в�
	double w = fabs(v) / (sqrt(r / p) * s0);
	// �жϸù۲�ֵ�Ƿ���ܺ��дֲ�
	if (w > 2.56) return true;
	return false;
};

// ����: power_Ld3
// ����: ѡȨ����ʹ�õ�Ȩ��������(ָ������)
//       LD3 = 1; ( sqrt(t) < K )
//       LD3 = pow(exp(-pow(t, 2.2)), 0.05) ( sqrt(t) > K, n = 2, 3 )
//       LD3 = pow(exp(-pow(t, 1.5)), 0.05) ( sqrt(t) > K, n = 4, 5, ... )
// ����: ������ t = v*v/(s0*s0*r)
//       v  --- �в�
//       s0 --- ��λȨ�����Ĺ���ֵ
//       r  --- ����۲����
//       n  --- ��������
//       K  --- �ٽ�ֵ, ��Ӧ��������ˮƽa0 = 1%, K = 2.56
inline double IterativePowerLd3(double v, double s0, double r, int n) {
	double t = v * v / (s0 * s0 * r);
	if (t <= 6.5536) return 1.0; // 6.5536 = 2.56*2.56;
	double a = (n < 4) ? pow(t, 2.2) : pow(t, 1.5);
	double p = pow(exp(-a), 0.05);
	if (p < 1.0e-05) p = 1.0e-05;
	return p;
};

// Givens transformation to the input matrix
inline void GivensTransformMatrix(double* a, double* b, int m, int n) {
	int i, j, k;
	double c, s, cs2, aij, akj, bi, bk;

	for (i = 0; i < n; i++) {
		for (k = i + 1; k < m; k++) {
			if (a[k * n + i] != 0.0) {
				cs2 = sqrt(a[i * n + i] * a[i * n + i] + a[k * n + i] * a[k * n + i]);
				c = a[i * n + i] / cs2;
				s = a[k * n + i] / cs2;

				for (j = i; j < n; j++) {
					aij = a[i * n + j];
					akj = a[k * n + j];
					a[i * n + j] = c * aij + s * akj;
					a[k * n + j] = -s * aij + c * akj;
				}

				bi = b[i];
				bk = b[k];
				b[i] = c * bi + s * bk;
				b[k] = -s * bi + c * bk;
			}
		}
	}
}

/****************************************************************************************
 * function: _atx_givens_direct_solve
 * 1. directly solve the error equation by means of the Givens transformation
 * 2. even the state of the error equation is very bad, the calculation is stable
 * [in] A: error equation parameters matrix (m*n)
 * [in] B: the residual vector (m*1)
 * [in] X: the unknown vector  (n*1)
 ****************************************************************************************/
inline int GivensDirectSolve(double* A, double* B, double* X, double* sigma, int m, int n) {
	int i, j;
	double* QA = new double[m * n];
	double* QB = new double[m];

	memcpy(QA, A, m * n * sizeof(double));
	memcpy(QB, B, m * 1 * sizeof(double));
	memset(X, 0, n * 1 * sizeof(double));

	GivensTransformMatrix(QA, QB, m, n);

	X[n - 1] = QB[n - 1] / QA[(n - 1) * n + n - 1];
	for (i = n - 2; i >= 0; i--) {
		X[i] = QB[i];
		for (j = i + 1; j < n; j++)
			X[i] -= QA[i * n + j] * X[j];

		if (QA[i * n + i] == 0.0) {
			printf("\nError: divided by zero !\n\n");
			delete[] QA;
			delete[] QB;
			memset(X, 0, n * 1 * sizeof(double));
			return 0;
		}

		X[i] /= QA[i * n + i];
	}

	double sum = 0.0;
	if (m > n) {
		for (i = n; i < m; i++)
			sum += QB[i] * QB[i];
		*sigma = sqrt(sum / (m - n));
	}
	else if (m == n) {
		*sigma = 0.0;
	}

	delete[] QA;
	delete[] QB;
	return 1;
};

#endif // !defined(AFX_INDIRECTADJUSTMENTX_H__4A7BD844_9392_49DF_9428_BD6C72DFA1E4__INCLUDED_)

