#ifndef true_modulo_H
#define true_modulo_H

inline int modulo(int a, int b)
{
  if (b == 0)
    return 0;
  if (b < 0)
    return -modulo(-a, -b);
  const int result = a % b;
  return result >= 0 ? result : result + b;
}

inline float modulo(float a, float b)
{
  if (b == 0)
    return 0;
  if (b < 0)
    return -modulo(-a, -b);
  int a2 = a * 10000;
  int b2 = b * 10000;
  const float result = (a2 % b2) * 0.0001;
  return result >= 0 ? result : result + b;
}

inline double modulo(double a, double b)
{
  if (b == 0)
    return 0;
  if (b < 0)
    return -modulo(-a, -b);
  int a2 = a * 10000;
  int b2 = b * 10000;
  const double result = (a2 % b2) * 0.0001;
  return result >= 0 ? result : result + b;
}
#endif