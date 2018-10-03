#include "temp_calc.h"
// Значение температуры, возвращаемое если сумма результатов АЦП больше первого значения таблицы
#define TEMPERATURE_UNDER -50
// Значение температуры, возвращаемое если сумма результатов АЦП меньше последнего значения таблицы
#define TEMPERATURE_OVER 1000
// Значение температуры соответствующее первому значению таблицы
#define TEMPERATURE_TABLE_START -50
// Шаг таблицы
#define TEMPERATURE_TABLE_STEP 10


// Метод доступа к элементу таблицы, должна соответствовать temperature_table_entry_type
#define TEMPERATURE_TABLE_READ(i) termo_table[i]

/* Таблица суммарного значения АЦП в зависимости от температуры. От большего значения к меньшему
   Для построения таблицы использованы следующие парамертры:
     R1(T1): 10кОм(25°С)
     Таблица R/T характеристик: EPCOS R/T:2904; B25/100:4300K
     Схема включения: A
     Ra: 5.1кОм
     Напряжения U0/Uref: 3.3В/3.3В
*/
const temperature_table_entry_type termo_table[]  = {
    3696, 3675, 3653, 3630, 3607, 3582, 3557, 3531,
    3504, 3476, 3448, 3418, 3388, 3356, 3324, 3291,
    3258, 3223, 3188, 3152, 3115, 3078, 3040, 3001,
    2961, 2921, 2881, 2839, 2797, 2755, 2713, 2669,
    2626, 2582, 2538, 2494, 2449, 2404, 2359, 2313,
    2268, 2223, 2177, 2132, 2087, 2043, 1999, 1955,
    1912, 1869, 1826, 1784, 1742, 1701, 1660, 1620,
    1580, 1540, 1502, 1464, 1426, 1389, 1353, 1318,
    1283, 1249, 1215, 1183, 1150, 1119, 1089, 1058,
    1029, 1001, 973, 945, 919, 893, 868, 843,
    819, 796, 773, 751, 730, 709, 689, 669,
    650, 632, 614, 597, 580, 563, 547, 532,
    516, 502, 487, 473, 460, 447, 434, 422,
    410, 398
};
// Функция вычисляет значение температуры в десятых долях градусов Цельсия
// в зависимости от суммарного значения АЦП.
int16_t calc_temperature(temperature_table_entry_type adcsum) {
  temperature_table_index_type l = 0;
  temperature_table_index_type r = (sizeof(termo_table) / sizeof(termo_table[0])) - 1;
  temperature_table_entry_type thigh = TEMPERATURE_TABLE_READ(r);
  temperature_table_entry_type tlow;
  temperature_table_entry_type vl;
  temperature_table_entry_type vr;
  temperature_table_entry_type vd;
	temperature_table_index_type m;
	temperature_table_entry_type mid;
  int16_t res;
  // Проверка выхода за пределы и граничных значений
  if (adcsum <= thigh) {
    #ifdef TEMPERATURE_UNDER
      if (adcsum < thigh) 
        return TEMPERATURE_OVER;//TEMPERATURE_UNDER;
    #endif
    return TEMPERATURE_TABLE_STEP * r + TEMPERATURE_TABLE_START;
  }
   tlow = TEMPERATURE_TABLE_READ(0);
  if (adcsum >= tlow) {
    #ifdef TEMPERATURE_OVER
      if (adcsum > tlow)
        return TEMPERATURE_UNDER;//TEMPERATURE_OVER;
    #endif
    return TEMPERATURE_TABLE_START;
  }

  // Двоичный поиск по таблице
  while ((r - l) > 1) {
     m = (l + r) >> 1;
     mid = TEMPERATURE_TABLE_READ(m);
    if (adcsum > mid) {
      r = m;
    } else {
      l = m;
    }
  }
    vl = TEMPERATURE_TABLE_READ(l);
  if (adcsum >= vl) {
    return l * TEMPERATURE_TABLE_STEP + TEMPERATURE_TABLE_START;
  }
   vr = TEMPERATURE_TABLE_READ(r);
    vd = vl - vr;
   res = TEMPERATURE_TABLE_START + r * TEMPERATURE_TABLE_STEP;
  if (vd) {
    // Линейная интерполяция
    res -= ((TEMPERATURE_TABLE_STEP * (int32_t )(adcsum - vr) + (vd >> 1)) / vd);
  }
  return res;
}
