#ifndef __TAPERING_WINDOW_H 
#define __TAPERING_WINDOW_H

float32_t tukey_window[1024] = {0.000000000000000, 0.000037722720717, 0.000150885190854, 0.000339470335226, 0.000603449698054, 0.000942783447259, 0.001357420380471, 0.001847297932759, 0.002412342186065, 0.003052467880363, 0.003767578426522, 0.004557565920881, 0.005422311161528, 0.006361683666292, 0.007375541692425, 0.008463732257996, 0.009626091164968, 0.010862443023980, 0.012172601280808, 0.013556368244517, 0.015013535117288, 0.016543882025924, 0.018147178055030, 0.019823181281852, 0.021571638812783, 0.023392286821524, 0.025284850588888, 0.027249044544258, 0.029284572308675, 0.031391126739555, 0.033568389977042, 0.035816033491964, 0.038133718135405, 0.040521094189885, 0.042977801422122, 0.045503469137393, 0.048097716235466, 0.050760151268108, 0.053490372498144, 0.056287967960084, 0.059152515522278, 0.062083582950616, 0.065080727973745, 0.068143498349806, 0.071271431934675, 0.074464056751689, 0.077720891062874, 0.081041443441622, 0.084425212846856, 0.087871688698621, 0.091380350955133, 0.094950670191248, 0.098582107678343, 0.102274115465610, 0.106026136462734, 0.109837604523954, 0.113707944533490, 0.117636572492321, 0.121622895606305, 0.125666312375629, 0.129766212685566, 0.133921977898538, 0.138132980947465, 0.142398586430378, 0.146718150706299, 0.151091021992362, 0.155516540462157, 0.159994038345296, 0.164522840028169, 0.169102262155893, 0.173731613735420, 0.178410196239802, 0.183137303713596, 0.187912222879380, 0.192734233245387, 0.197602607214214, 0.202516610192617, 0.207475500702346, 0.212478530492035, 0.217524944650102, 0.222613981718660, 0.227744873808411, 0.232916846714519, 0.238129120033426, 0.243380907280608, 0.248671416009252, 0.253999847929824, 0.259365399030529, 0.264767259698622, 0.270204614842580, 0.275676644015082, 0.281182521536818, 0.286721416621066, 0.292292493499057, 0.297894911546082, 0.303527825408337, 0.309190385130475, 0.314881736283861, 0.320601020095493, 0.326347373577590, 0.332119929657800, 0.337917817310040, 0.343740161685923, 0.349586084246766, 0.355454702896154, 0.361345132113036, 0.367256483085347, 0.373187863844121, 0.379138379398078, 0.385107131868672, 0.391093220625574, 0.397095742422567, 0.403113791533836, 0.409146459890638, 0.415192837218320, 0.421252011173667, 0.427323067482572, 0.433405090077989, 0.439497161238159, 0.445598361725086, 0.451707770923241, 0.457824466978478, 0.463947526937127, 0.470076026885267, 0.476209042088129, 0.482345647129634, 0.488484916052031, 0.494625922495611, 0.500767739838490, 0.506909441336427, 0.513050100262661, 0.519188790047744, 0.525324584419356, 0.531456557542067, 0.537583784157038, 0.543705339721636, 0.549820300548938, 0.555927743947105, 0.562026748358610, 0.568116393499295, 0.574195760497226, 0.580263932031351, 0.586319992469909, 0.592363028008595, 0.598392126808439, 0.604406379133402, 0.610404877487641, 0.616386716752442, 0.622350994322798, 0.628296810243601, 0.634223267345437, 0.640129471379962, 0.646014531154834, 0.651877558668189, 0.657717669242628, 0.663533981658711, 0.669325618287922, 0.675091705225098, 0.680831372420289, 0.686543753810046, 0.692227987448096, 0.697883215635408, 0.703508585049608, 0.709103246873738, 0.714666356924337, 0.720197075778817, 0.725694568902128, 0.731158006772678, 0.736586565007505, 0.741979424486662, 0.747335771476822, 0.752654797754059, 0.757935700725802, 0.763177683551939, 0.768379955265053, 0.773541730889774, 0.778662231561218, 0.783740684642520, 0.788776323841411, 0.793768389325845, 0.798716127838655, 0.803618792811209, 0.808475644476059, 0.813285949978571, 0.818048983487500, 0.822764026304515, 0.827430366972643, 0.832047301383620, 0.836614132884137, 0.841130172380956, 0.845594738444892, 0.850007157413630, 0.854366763493374, 0.858672898859316, 0.862924913754887, 0.867122166589807, 0.871264024036890, 0.875349861127608, 0.879379061346395, 0.883351016723675, 0.887265127927591, 0.891120804354451, 0.894917464217833, 0.898654534636379, 0.902331451720234, 0.905947660656133, 0.909502615791117, 0.912995780714867, 0.916426628340643, 0.919794640984819, 0.923099310444994, 0.926340138076674, 0.929516634868517, 0.932628321516118, 0.935674728494331, 0.938655396128118, 0.941569874661908, 0.944417724327462, 0.947198515410229, 0.949911828314189, 0.952557253625161, 0.955134392172584, 0.957642855089749, 0.960082263872470, 0.962452250436204, 0.964752457171584, 0.966982536998386, 0.969142153417897, 0.971230980563686, 0.973248703250784, 0.975195017023231, 0.977069628200024, 0.978872253919429, 0.980602622181658, 0.982260471889917, 0.983845552889800, 0.985357626007035, 0.986796463083574, 0.988161847012021, 0.989453571768390, 0.990671442443191, 0.991815275270844, 0.992884897657402, 0.993880148206600, 0.994800876744203, 0.995646944340671, 0.996418223332115, 0.997114597339568, 0.997735961286542, 0.998282221414882, 0.998753295298914, 0.999149111857885, 0.999469611366685, 0.999714745464859, 0.999884477163907, 0.999978780852864, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 0.999978780852864, 0.999884477163907, 0.999714745464859, 0.999469611366685, 0.999149111857885, 0.998753295298914, 0.998282221414882, 0.997735961286542, 0.997114597339568, 0.996418223332115, 0.995646944340671, 0.994800876744203, 0.993880148206600, 0.992884897657402, 0.991815275270844, 0.990671442443191, 0.989453571768390, 0.988161847012021, 0.986796463083574, 0.985357626007035, 0.983845552889800, 0.982260471889917, 0.980602622181658, 0.978872253919429, 0.977069628200024, 0.975195017023231, 0.973248703250784, 0.971230980563686, 0.969142153417897, 0.966982536998386, 0.964752457171584, 0.962452250436204, 0.960082263872471, 0.957642855089749, 0.955134392172584, 0.952557253625161, 0.949911828314189, 0.947198515410229, 0.944417724327462, 0.941569874661908, 0.938655396128118, 0.935674728494331, 0.932628321516118, 0.929516634868517, 0.926340138076674, 0.923099310444994, 0.919794640984820, 0.916426628340643, 0.912995780714867, 0.909502615791117, 0.905947660656133, 0.902331451720234, 0.898654534636379, 0.894917464217833, 0.891120804354451, 0.887265127927592, 0.883351016723675, 0.879379061346396, 0.875349861127608, 0.871264024036890, 0.867122166589808, 0.862924913754888, 0.858672898859316, 0.854366763493374, 0.850007157413629, 0.845594738444892, 0.841130172380956, 0.836614132884136, 0.832047301383619, 0.827430366972642, 0.822764026304515, 0.818048983487500, 0.813285949978571, 0.808475644476059, 0.803618792811208, 0.798716127838655, 0.793768389325845, 0.788776323841411, 0.783740684642520, 0.778662231561218, 0.773541730889773, 0.768379955265053, 0.763177683551939, 0.757935700725802, 0.752654797754059, 0.747335771476822, 0.741979424486662, 0.736586565007504, 0.731158006772678, 0.725694568902128, 0.720197075778817, 0.714666356924337, 0.709103246873738, 0.703508585049608, 0.697883215635408, 0.692227987448096, 0.686543753810046, 0.680831372420289, 0.675091705225098, 0.669325618287922, 0.663533981658711, 0.657717669242628, 0.651877558668188, 0.646014531154834, 0.640129471379962, 0.634223267345437, 0.628296810243601, 0.622350994322798, 0.616386716752442, 0.610404877487640, 0.604406379133402, 0.598392126808439, 0.592363028008595, 0.586319992469909, 0.580263932031351, 0.574195760497226, 0.568116393499295, 0.562026748358610, 0.555927743947105, 0.549820300548938, 0.543705339721636, 0.537583784157038, 0.531456557542067, 0.525324584419356, 0.519188790047744, 0.513050100262661, 0.506909441336427, 0.500767739838490, 0.494625922495611, 0.488484916052031, 0.482345647129634, 0.476209042088129, 0.470076026885267, 0.463947526937127, 0.457824466978478, 0.451707770923241, 0.445598361725086, 0.439497161238159, 0.433405090077989, 0.427323067482572, 0.421252011173667, 0.415192837218320, 0.409146459890638, 0.403113791533836, 0.397095742422567, 0.391093220625574, 0.385107131868672, 0.379138379398078, 0.373187863844121, 0.367256483085347, 0.361345132113036, 0.355454702896154, 0.349586084246767, 0.343740161685923, 0.337917817310040, 0.332119929657800, 0.326347373577590, 0.320601020095494, 0.314881736283861, 0.309190385130475, 0.303527825408337, 0.297894911546083, 0.292292493499057, 0.286721416621066, 0.281182521536818, 0.275676644015082, 0.270204614842580, 0.264767259698622, 0.259365399030529, 0.253999847929825, 0.248671416009252, 0.243380907280608, 0.238129120033426, 0.232916846714519, 0.227744873808411, 0.222613981718660, 0.217524944650103, 0.212478530492036, 0.207475500702346, 0.202516610192617, 0.197602607214215, 0.192734233245387, 0.187912222879380, 0.183137303713596, 0.178410196239802, 0.173731613735420, 0.169102262155893, 0.164522840028169, 0.159994038345296, 0.155516540462157, 0.151091021992362, 0.146718150706299, 0.142398586430377, 0.138132980947465, 0.133921977898538, 0.129766212685566, 0.125666312375629, 0.121622895606305, 0.117636572492321, 0.113707944533490, 0.109837604523954, 0.106026136462733, 0.102274115465609, 0.098582107678343, 0.094950670191248, 0.091380350955133, 0.087871688698621, 0.084425212846855, 0.081041443441622, 0.077720891062873, 0.074464056751689, 0.071271431934674, 0.068143498349806, 0.065080727973744, 0.062083582950616, 0.059152515522278, 0.056287967960084, 0.053490372498144, 0.050760151268108, 0.048097716235466, 0.045503469137392, 0.042977801422121, 0.040521094189885, 0.038133718135405, 0.035816033491964, 0.033568389977042, 0.031391126739555, 0.029284572308675, 0.027249044544258, 0.025284850588888, 0.023392286821524, 0.021571638812783, 0.019823181281852, 0.018147178055030, 0.016543882025924, 0.015013535117288, 0.013556368244517, 0.012172601280808, 0.010862443023980, 0.009626091164968, 0.008463732257996, 0.007375541692425, 0.006361683666292, 0.005422311161528, 0.004557565920881, 0.003767578426522, 0.003052467880363, 0.002412342186065, 0.001847297932759, 0.001357420380471, 0.000942783447259, 0.000603449698054, 0.000339470335226, 0.000150885190854, 0.000037722720717, 0.000000000000000}; 

#endif /* __FFT_BIN_DATA_H */