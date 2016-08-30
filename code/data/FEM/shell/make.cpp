#include <cstdio>
#inlcude <algorithm>

using namespace std;

double Lx = - 5.0;
double Ly = 1.0;
double Rx = 5.0;
double ry = 11.0;
int N = 100;
int M = 100;

int main()
{
	freopen("cloth.shell", "w", stdout);
	printf("%d %d\n", N * M, 2 * (N - 1) * (M - 1));
	for (int i=0;i<N;++i)
		for (int j=0;j<M;++j)
		{
			double x = Lx + (Rx - Lx) * i / N;
			double y = Ly + (Ry - Ly) * j / M;
			printf("%.4lf %.4lf 0\n", x, y, 0);
		}
	for (int i=0;i<N-1;++i)
		for (int j=0;j<M-1;++j)
		{
			printf("%d %d %d\n", i * M + j, (i + 1) * M + j + 1, i * M + j + 1);
			printf("%d %d %d\n", i * M + j, (i + 1) * M + j + 1, (i + 1) * M + j);
		}
	
	return 0;
}