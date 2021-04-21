#ifndef __HANN_WINDOW_H 
#define __HANN_WINDOW_H

int16_t hann_window[2048] = {0, 0, 0, 1, 1, 2, 3, 4, 5, 6, 8, 9, 11, 13, 15, 17, 20, 22, 25, 28, 31, 34, 37, 41, 44, 48, 52, 56, 60, 65, 69, 74, 79, 84, 89, 94, 100, 106, 111, 117, 123, 130, 136, 142, 149, 156, 163, 170, 177, 185, 193, 200, 208, 216, 225, 233, 241, 250, 259, 268, 277, 286, 296, 305, 315, 325, 335, 345, 356, 366, 377, 388, 398, 410, 421, 432, 444, 455, 467, 479, 491, 504, 516, 529, 542, 554, 568, 581, 594, 608, 621, 635, 649, 663, 677, 692, 706, 721, 736, 751, 766, 781, 796, 812, 828, 844, 860, 876, 892, 908, 925, 942, 959, 976, 993, 1010, 1028, 1045, 1063, 1081, 1099, 1117, 1135, 1154, 1172, 1191, 1210, 1229, 1248, 1268, 1287, 1307, 1326, 1346, 1366, 1387, 1407, 1427, 1448, 1469, 1490, 1511, 1532, 1553, 1575, 1596, 1618, 1640, 1662, 1684, 1706, 1728, 1751, 1774, 1797, 1820, 1843, 1866, 1889, 1913, 1936, 1960, 1984, 2008, 2032, 2057, 2081, 2106, 2130, 2155, 2180, 2205, 2231, 2256, 2282, 2307, 2333, 2359, 2385, 2411, 2438, 2464, 2491, 2517, 2544, 2571, 2598, 2626, 2653, 2680, 2708, 2736, 2764, 2792, 2820, 2848, 2877, 2905, 2934, 2963, 2991, 3020, 3050, 3079, 3108, 3138, 3168, 3197, 3227, 3257, 3287, 3318, 3348, 3379, 3409, 3440, 3471, 3502, 3533, 3564, 3596, 3627, 3659, 3691, 3722, 3754, 3786, 3819, 3851, 3883, 3916, 3949, 3981, 4014, 4047, 4081, 4114, 4147, 4181, 4214, 4248, 4282, 4316, 4350, 4384, 4418, 4453, 4487, 4522, 4557, 4592, 4627, 4662, 4697, 4732, 4768, 4803, 4839, 4874, 4910, 4946, 4982, 5018, 5055, 5091, 5128, 5164, 5201, 5238, 5275, 5312, 5349, 5386, 5423, 5461, 5498, 5536, 5574, 5611, 5649, 5687, 5726, 5764, 5802, 5841, 5879, 5918, 5957, 5995, 6034, 6073, 6112, 6152, 6191, 6230, 6270, 6310, 6349, 6389, 6429, 6469, 6509, 6549, 6589, 6630, 6670, 6711, 6751, 6792, 6833, 6874, 6915, 6956, 6997, 7038, 7080, 7121, 7163, 7204, 7246, 7288, 7330, 7372, 7414, 7456, 7498, 7540, 7583, 7625, 7668, 7710, 7753, 7796, 7839, 7882, 7925, 7968, 8011, 8054, 8098, 8141, 8184, 8228, 8272, 8315, 8359, 8403, 8447, 8491, 8535, 8579, 8624, 8668, 8712, 8757, 8801, 8846, 8891, 8935, 8980, 9025, 9070, 9115, 9160, 9205, 9251, 9296, 9341, 9387, 9432, 9478, 9524, 9569, 9615, 9661, 9707, 9753, 9799, 9845, 9891, 9937, 9983, 10030, 10076, 10123, 10169, 10216, 10262, 10309, 10356, 10402, 10449, 10496, 10543, 10590, 10637, 10684, 10731, 10779, 10826, 10873, 10921, 10968, 11016, 11063, 11111, 11158, 11206, 11254, 11302, 11349, 11397, 11445, 11493, 11541, 11589, 11637, 11686, 11734, 11782, 11830, 11879, 11927, 11975, 12024, 12072, 12121, 12169, 12218, 12267, 12315, 12364, 12413, 12462, 12511, 12559, 12608, 12657, 12706, 12755, 12804, 12853, 12903, 12952, 13001, 13050, 13099, 13149, 13198, 13247, 13297, 13346, 13396, 13445, 13495, 13544, 13594, 13643, 13693, 13742, 13792, 13842, 13891, 13941, 13991, 14041, 14090, 14140, 14190, 14240, 14290, 14340, 14389, 14439, 14489, 14539, 14589, 14639, 14689, 14739, 14789, 14839, 14889, 14940, 14990, 15040, 15090, 15140, 15190, 15240, 15291, 15341, 15391, 15441, 15491, 15542, 15592, 15642, 15692, 15742, 15793, 15843, 15893, 15944, 15994, 16044, 16094, 16145, 16195, 16245, 16295, 16346, 16396, 16446, 16497, 16547, 16597, 16648, 16698, 16748, 16798, 16849, 16899, 16949, 16999, 17050, 17100, 17150, 17200, 17251, 17301, 17351, 17401, 17451, 17502, 17552, 17602, 17652, 17702, 17752, 17802, 17852, 17903, 17953, 18003, 18053, 18103, 18153, 18203, 18253, 18303, 18353, 18402, 18452, 18502, 18552, 18602, 18652, 18702, 18751, 18801, 18851, 18900, 18950, 19000, 19049, 19099, 19149, 19198, 19248, 19297, 19347, 19396, 19446, 19495, 19544, 19594, 19643, 19692, 19741, 19791, 19840, 19889, 19938, 19987, 20036, 20085, 20134, 20183, 20232, 20281, 20330, 20378, 20427, 20476, 20525, 20573, 20622, 20670, 20719, 20767, 20816, 20864, 20913, 20961, 21009, 21057, 21106, 21154, 21202, 21250, 21298, 21346, 21394, 21441, 21489, 21537, 21585, 21632, 21680, 21728, 21775, 21823, 21870, 21917, 21965, 22012, 22059, 22106, 22153, 22200, 22247, 22294, 22341, 22388, 22435, 22481, 22528, 22575, 22621, 22668, 22714, 22760, 22807, 22853, 22899, 22945, 22991, 23037, 23083, 23129, 23175, 23221, 23266, 23312, 23357, 23403, 23448, 23494, 23539, 23584, 23629, 23674, 23719, 23764, 23809, 23854, 23899, 23943, 23988, 24032, 24077, 24121, 24165, 24210, 24254, 24298, 24342, 24386, 24430, 24473, 24517, 24561, 24604, 24648, 24691, 24734, 24778, 24821, 24864, 24907, 24950, 24993, 25035, 25078, 25121, 25163, 25205, 25248, 25290, 25332, 25374, 25416, 25458, 25500, 25542, 25583, 25625, 25666, 25708, 25749, 25790, 25832, 25873, 25914, 25954, 25995, 26036, 26076, 26117, 26157, 26198, 26238, 26278, 26318, 26358, 26398, 26438, 26477, 26517, 26556, 26596, 26635, 26674, 26713, 26752, 26791, 26830, 26869, 26907, 26946, 26984, 27022, 27060, 27099, 27137, 27174, 27212, 27250, 27288, 27325, 27362, 27400, 27437, 27474, 27511, 27548, 27584, 27621, 27658, 27694, 27730, 27767, 27803, 27839, 27875, 27910, 27946, 27982, 28017, 28053, 28088, 28123, 28158, 28193, 28228, 28262, 28297, 28331, 28366, 28400, 28434, 28468, 28502, 28536, 28569, 28603, 28636, 28670, 28703, 28736, 28769, 28802, 28835, 28867, 28900, 28932, 28964, 28997, 29029, 29061, 29092, 29124, 29156, 29187, 29218, 29250, 29281, 29312, 29342, 29373, 29404, 29434, 29464, 29495, 29525, 29555, 29585, 29614, 29644, 29673, 29703, 29732, 29761, 29790, 29819, 29848, 29876, 29905, 29933, 29961, 29989, 30017, 30045, 30073, 30100, 30128, 30155, 30182, 30209, 30236, 30263, 30290, 30316, 30343, 30369, 30395, 30421, 30447, 30473, 30498, 30524, 30549, 30574, 30599, 30624, 30649, 30674, 30698, 30723, 30747, 30771, 30795, 30819, 30842, 30866, 30889, 30913, 30936, 30959, 30982, 31005, 31027, 31050, 31072, 31094, 31116, 31138, 31160, 31182, 31203, 31225, 31246, 31267, 31288, 31309, 31329, 31350, 31370, 31391, 31411, 31431, 31450, 31470, 31490, 31509, 31528, 31547, 31566, 31585, 31604, 31622, 31641, 31659, 31677, 31695, 31713, 31731, 31748, 31766, 31783, 31800, 31817, 31834, 31850, 31867, 31883, 31899, 31915, 31931, 31947, 31963, 31978, 31994, 32009, 32024, 32039, 32054, 32068, 32083, 32097, 32111, 32125, 32139, 32153, 32166, 32180, 32193, 32206, 32219, 32232, 32244, 32257, 32269, 32282, 32294, 32306, 32317, 32329, 32340, 32352, 32363, 32374, 32385, 32396, 32406, 32417, 32427, 32437, 32447, 32457, 32466, 32476, 32485, 32495, 32504, 32512, 32521, 32530, 32538, 32547, 32555, 32563, 32571, 32578, 32586, 32593, 32600, 32607, 32614, 32621, 32628, 32634, 32641, 32647, 32653, 32659, 32664, 32670, 32675, 32680, 32686, 32690, 32695, 32700, 32704, 32709, 32713, 32717, 32721, 32724, 32728, 32731, 32735, 32738, 32741, 32743, 32746, 32748, 32751, 32753, 32755, 32757, 32758, 32760, 32761, 32763, 32764, 32765, 32765, 32766, 32767, 32767, 32767, 32767, 32767, 32767, 32766, 32765, 32765, 32764, 32763, 32761, 32760, 32758, 32757, 32755, 32753, 32751, 32748, 32746, 32743, 32741, 32738, 32735, 32731, 32728, 32724, 32721, 32717, 32713, 32709, 32704, 32700, 32695, 32690, 32686, 32680, 32675, 32670, 32664, 32659, 32653, 32647, 32641, 32634, 32628, 32621, 32614, 32607, 32600, 32593, 32586, 32578, 32571, 32563, 32555, 32547, 32538, 32530, 32521, 32512, 32504, 32495, 32485, 32476, 32466, 32457, 32447, 32437, 32427, 32417, 32406, 32396, 32385, 32374, 32363, 32352, 32340, 32329, 32317, 32306, 32294, 32282, 32269, 32257, 32244, 32232, 32219, 32206, 32193, 32180, 32166, 32153, 32139, 32125, 32111, 32097, 32083, 32068, 32054, 32039, 32024, 32009, 31994, 31978, 31963, 31947, 31931, 31915, 31899, 31883, 31867, 31850, 31834, 31817, 31800, 31783, 31766, 31748, 31731, 31713, 31695, 31677, 31659, 31641, 31622, 31604, 31585, 31566, 31547, 31528, 31509, 31490, 31470, 31450, 31431, 31411, 31391, 31370, 31350, 31329, 31309, 31288, 31267, 31246, 31225, 31203, 31182, 31160, 31138, 31116, 31094, 31072, 31050, 31027, 31005, 30982, 30959, 30936, 30913, 30889, 30866, 30842, 30819, 30795, 30771, 30747, 30723, 30698, 30674, 30649, 30624, 30599, 30574, 30549, 30524, 30498, 30473, 30447, 30421, 30395, 30369, 30343, 30316, 30290, 30263, 30236, 30209, 30182, 30155, 30128, 30100, 30073, 30045, 30017, 29989, 29961, 29933, 29905, 29876, 29848, 29819, 29790, 29761, 29732, 29703, 29673, 29644, 29614, 29585, 29555, 29525, 29495, 29464, 29434, 29404, 29373, 29342, 29312, 29281, 29250, 29218, 29187, 29156, 29124, 29092, 29061, 29029, 28997, 28964, 28932, 28900, 28867, 28835, 28802, 28769, 28736, 28703, 28670, 28636, 28603, 28569, 28536, 28502, 28468, 28434, 28400, 28366, 28331, 28297, 28262, 28228, 28193, 28158, 28123, 28088, 28053, 28017, 27982, 27946, 27910, 27875, 27839, 27803, 27767, 27730, 27694, 27658, 27621, 27584, 27548, 27511, 27474, 27437, 27400, 27362, 27325, 27288, 27250, 27212, 27174, 27137, 27099, 27060, 27022, 26984, 26946, 26907, 26869, 26830, 26791, 26752, 26713, 26674, 26635, 26596, 26556, 26517, 26477, 26438, 26398, 26358, 26318, 26278, 26238, 26198, 26157, 26117, 26076, 26036, 25995, 25954, 25914, 25873, 25832, 25790, 25749, 25708, 25666, 25625, 25583, 25542, 25500, 25458, 25416, 25374, 25332, 25290, 25248, 25205, 25163, 25121, 25078, 25035, 24993, 24950, 24907, 24864, 24821, 24778, 24734, 24691, 24648, 24604, 24561, 24517, 24473, 24430, 24386, 24342, 24298, 24254, 24210, 24165, 24121, 24077, 24032, 23988, 23943, 23899, 23854, 23809, 23764, 23719, 23674, 23629, 23584, 23539, 23494, 23448, 23403, 23357, 23312, 23266, 23221, 23175, 23129, 23083, 23037, 22991, 22945, 22899, 22853, 22807, 22760, 22714, 22668, 22621, 22575, 22528, 22481, 22435, 22388, 22341, 22294, 22247, 22200, 22153, 22106, 22059, 22012, 21965, 21917, 21870, 21823, 21775, 21728, 21680, 21632, 21585, 21537, 21489, 21441, 21394, 21346, 21298, 21250, 21202, 21154, 21106, 21057, 21009, 20961, 20913, 20864, 20816, 20767, 20719, 20670, 20622, 20573, 20525, 20476, 20427, 20378, 20330, 20281, 20232, 20183, 20134, 20085, 20036, 19987, 19938, 19889, 19840, 19791, 19741, 19692, 19643, 19594, 19544, 19495, 19446, 19396, 19347, 19297, 19248, 19198, 19149, 19099, 19049, 19000, 18950, 18900, 18851, 18801, 18751, 18702, 18652, 18602, 18552, 18502, 18452, 18402, 18353, 18303, 18253, 18203, 18153, 18103, 18053, 18003, 17953, 17903, 17852, 17802, 17752, 17702, 17652, 17602, 17552, 17502, 17451, 17401, 17351, 17301, 17251, 17200, 17150, 17100, 17050, 16999, 16949, 16899, 16849, 16798, 16748, 16698, 16648, 16597, 16547, 16497, 16446, 16396, 16346, 16295, 16245, 16195, 16145, 16094, 16044, 15994, 15944, 15893, 15843, 15793, 15742, 15692, 15642, 15592, 15542, 15491, 15441, 15391, 15341, 15291, 15240, 15190, 15140, 15090, 15040, 14990, 14940, 14889, 14839, 14789, 14739, 14689, 14639, 14589, 14539, 14489, 14439, 14389, 14340, 14290, 14240, 14190, 14140, 14090, 14041, 13991, 13941, 13891, 13842, 13792, 13742, 13693, 13643, 13594, 13544, 13495, 13445, 13396, 13346, 13297, 13247, 13198, 13149, 13099, 13050, 13001, 12952, 12903, 12853, 12804, 12755, 12706, 12657, 12608, 12559, 12511, 12462, 12413, 12364, 12315, 12267, 12218, 12169, 12121, 12072, 12024, 11975, 11927, 11879, 11830, 11782, 11734, 11686, 11637, 11589, 11541, 11493, 11445, 11397, 11349, 11302, 11254, 11206, 11158, 11111, 11063, 11016, 10968, 10921, 10873, 10826, 10779, 10731, 10684, 10637, 10590, 10543, 10496, 10449, 10402, 10356, 10309, 10262, 10216, 10169, 10123, 10076, 10030, 9983, 9937, 9891, 9845, 9799, 9753, 9707, 9661, 9615, 9569, 9524, 9478, 9432, 9387, 9341, 9296, 9251, 9205, 9160, 9115, 9070, 9025, 8980, 8935, 8891, 8846, 8801, 8757, 8712, 8668, 8624, 8579, 8535, 8491, 8447, 8403, 8359, 8315, 8272, 8228, 8184, 8141, 8098, 8054, 8011, 7968, 7925, 7882, 7839, 7796, 7753, 7710, 7668, 7625, 7583, 7540, 7498, 7456, 7414, 7372, 7330, 7288, 7246, 7204, 7163, 7121, 7080, 7038, 6997, 6956, 6915, 6874, 6833, 6792, 6751, 6711, 6670, 6630, 6589, 6549, 6509, 6469, 6429, 6389, 6349, 6310, 6270, 6230, 6191, 6152, 6112, 6073, 6034, 5995, 5957, 5918, 5879, 5841, 5802, 5764, 5726, 5687, 5649, 5611, 5574, 5536, 5498, 5461, 5423, 5386, 5349, 5312, 5275, 5238, 5201, 5164, 5128, 5091, 5055, 5018, 4982, 4946, 4910, 4874, 4839, 4803, 4768, 4732, 4697, 4662, 4627, 4592, 4557, 4522, 4487, 4453, 4418, 4384, 4350, 4316, 4282, 4248, 4214, 4181, 4147, 4114, 4081, 4047, 4014, 3981, 3949, 3916, 3883, 3851, 3819, 3786, 3754, 3722, 3691, 3659, 3627, 3596, 3564, 3533, 3502, 3471, 3440, 3409, 3379, 3348, 3318, 3287, 3257, 3227, 3197, 3168, 3138, 3108, 3079, 3050, 3020, 2991, 2963, 2934, 2905, 2877, 2848, 2820, 2792, 2764, 2736, 2708, 2680, 2653, 2626, 2598, 2571, 2544, 2517, 2491, 2464, 2438, 2411, 2385, 2359, 2333, 2307, 2282, 2256, 2231, 2205, 2180, 2155, 2130, 2106, 2081, 2057, 2032, 2008, 1984, 1960, 1936, 1913, 1889, 1866, 1843, 1820, 1797, 1774, 1751, 1728, 1706, 1684, 1662, 1640, 1618, 1596, 1575, 1553, 1532, 1511, 1490, 1469, 1448, 1427, 1407, 1387, 1366, 1346, 1326, 1307, 1287, 1268, 1248, 1229, 1210, 1191, 1172, 1154, 1135, 1117, 1099, 1081, 1063, 1045, 1028, 1010, 993, 976, 959, 942, 925, 908, 892, 876, 860, 844, 828, 812, 796, 781, 766, 751, 736, 721, 706, 692, 677, 663, 649, 635, 621, 608, 594, 581, 568, 554, 542, 529, 516, 504, 491, 479, 467, 455, 444, 432, 421, 410, 398, 388, 377, 366, 356, 345, 335, 325, 315, 305, 296, 286, 277, 268, 259, 250, 241, 233, 225, 216, 208, 200, 193, 185, 177, 170, 163, 156, 149, 142, 136, 130, 123, 117, 111, 106, 100, 94, 89, 84, 79, 74, 69, 65, 60, 56, 52, 48, 44, 41, 37, 34, 31, 28, 25, 22, 20, 17, 15, 13, 11, 9, 8, 6, 5, 4, 3, 2, 1, 1, 0, 0, 0}; 

#endif /* __HANN_WINDOW_H */