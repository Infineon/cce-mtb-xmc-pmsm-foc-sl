/**
 * @file pmsm_foc_svmsinetable.c
 * @Firmware PMSM_FOC_SL_XMC13_XMC14_V1_5
 * @Modified date: 2019-01-10
 *
 * @cond
 ****************************************
 * PMSM FOC Motor Control Library
 *
 * Copyright (c) 2015-2019, Infineon Technologies AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright notice, this list of conditions and the  following
 *   disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *   following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *   Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 *   products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * To improve the quality of the software, users are encouraged to share modifications, enhancements or bug fixes
 * with Infineon Technologies AG (dave@infineon.com).
 ******************************************
 *
 * @file pmsm_foc_svmsinetable.c
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 * @endcond
 *
 */
/*********************************************************************************************************************
 * HEADER FILES
 ***************************************/

#include "../Configuration/pmsm_foc_variables_scaling.h"

/*********************************************************************************************************************
 * MACROS
 ****************************************/

#define scale(C) ((uint16_t)(C * 1U))

/*********************************************************************************************************************
 * DATA STRUCTURES
 ***************************************/

/*
 * This is the sine Look-Up Table (LUT) for SVM calculations. Instead, can use CORDIC for the same calculations
 * (save >0.4kbyte Flash).
 * It contains angles of 0� to 60�. Array size 256 or 1024. Angle resolution is 60�/256 = 0.234� or 60�/1024 = 0.0586�.
 */

const uint16_t Sinus60_tab[]=
{
		0U, scale(33),scale(67),scale(100),scale(134),
		scale(167),scale(201),scale(234),scale(268),
		scale(301),scale(335),scale(368),scale(402),
		scale(435),scale(469),scale(502),scale(536),
		scale(569),scale(603),scale(636),scale(670),
		scale(703),scale(737),scale(770),scale(804),
		scale(837),scale(871),scale(904),scale(938),
		scale(971),scale(1005),scale(1038),scale(1072),
		scale(1105),scale(1139),scale(1172),scale(1206),
		scale(1239),scale(1273),scale(1306),scale(1339),
		scale(1373),scale(1406),scale(1440),scale(1473),
		scale(1507),scale(1540),scale(1574),scale(1607),
		scale(1641),scale(1674),scale(1708),scale(1741),
		scale(1775),scale(1808),scale(1842),scale(1875),
		scale(1908),scale(1942),scale(1975),scale(2009),
		scale(2042),scale(2076),scale(2109),scale(2143),
		scale(2176),scale(2209),scale(2243),scale(2276),
		scale(2310),scale(2343),scale(2377),scale(2410),
		scale(2443),scale(2477),scale(2510),scale(2544),
		scale(2577),scale(2610),scale(2644),scale(2677),
		scale(2711),scale(2744),scale(2777),scale(2811),
		scale(2844),scale(2878),scale(2911),scale(2944),
		scale(2978),scale(3011),scale(3044),scale(3078),
		scale(3111),scale(3145),scale(3178),scale(3211),
		scale(3245),scale(3278),scale(3311),scale(3345),
		scale(3378),scale(3411),scale(3445),scale(3478),
		scale(3511),scale(3545),scale(3578),scale(3611),
		scale(3644),scale(3678),scale(3711),scale(3744),
		scale(3778),scale(3811),scale(3844),scale(3877),
		scale(3911),scale(3944),scale(3977),scale(4011),
		scale(4044),scale(4077),scale(4110),scale(4144),
		scale(4177),scale(4210),scale(4243),scale(4276),
		scale(4310),scale(4343),scale(4376),scale(4409),
		scale(4443),scale(4476),scale(4509),scale(4542),
		scale(4575),scale(4608),scale(4642),scale(4675),
		scale(4708),scale(4741),scale(4774),scale(4807),
		scale(4841),scale(4874),scale(4907),scale(4940),
		scale(4973),scale(5006),scale(5039),scale(5072),
		scale(5106),scale(5139),scale(5172),scale(5205),
		scale(5238),scale(5271),scale(5304),scale(5337),
		scale(5370),scale(5403),scale(5436),scale(5469),
		scale(5502),scale(5535),scale(5568),scale(5601),
		scale(5634),scale(5667),scale(5700),scale(5733),
		scale(5766),scale(5799),scale(5832),scale(5865),
		scale(5898),scale(5931),scale(5964),scale(5997),
		scale(6030),scale(6063),scale(6096),scale(6129),
		scale(6162),scale(6195),scale(6228),scale(6261),
		scale(6293),scale(6326),scale(6359),scale(6392),
		scale(6425),scale(6458),scale(6491),scale(6523),
		scale(6556),scale(6589),scale(6622),scale(6655),
		scale(6688),scale(6720),scale(6753),scale(6786),
		scale(6819),scale(6851),scale(6884),scale(6917),
		scale(6950),scale(6982),scale(7015),scale(7048),
		scale(7081),scale(7113),scale(7146),scale(7179),
		scale(7211),scale(7244),scale(7277),scale(7310),
		scale(7342),scale(7375),scale(7407),scale(7440),
		scale(7473),scale(7505),scale(7538),scale(7571),
		scale(7603),scale(7636),scale(7668),scale(7701),
		scale(7733),scale(7766),scale(7799),scale(7831),
		scale(7864),scale(7896),scale(7929),scale(7961),
		scale(7994),scale(8026),scale(8059),scale(8091),
		scale(8124),scale(8156),scale(8189),scale(8221),
		scale(8253),scale(8286),scale(8318),scale(8351),
		scale(8383),scale(8415),scale(8448),scale(8480),
		scale(8513),scale(8545),scale(8577),scale(8610),
		scale(8642),scale(8674),scale(8707),scale(8739),
		scale(8771),scale(8803),scale(8836),scale(8868),
		scale(8900),scale(8932),scale(8965),scale(8997),
		scale(9029),scale(9061),scale(9094),scale(9126),
		scale(9158),scale(9190),scale(9222),scale(9254),
		scale(9287),scale(9319),scale(9351),scale(9383),
		scale(9415),scale(9447),scale(9479),scale(9511),
		scale(9543),scale(9575),scale(9607),scale(9639),
		scale(9671),scale(9703),scale(9735),scale(9767),
		scale(9799),scale(9831),scale(9863),scale(9895),
		scale(9927),scale(9959),scale(9991),scale(10023),
		scale(10055),scale(10087),scale(10119),scale(10151),
		scale(10182),scale(10214),scale(10246),scale(10278),
		scale(10310),scale(10342),scale(10373),scale(10405),
		scale(10437),scale(10469),scale(10500),scale(10532),
		scale(10564),scale(10596),scale(10627),scale(10659),
		scale(10691),scale(10722),scale(10754),scale(10786),
		scale(10817),scale(10849),scale(10880),scale(10912),
		scale(10944),scale(10975),scale(11007),scale(11038),
		scale(11070),scale(11101),scale(11133),scale(11164),
		scale(11196),scale(11227),scale(11259),scale(11290),
		scale(11322),scale(11353),scale(11385),scale(11416),
		scale(11448),scale(11479),scale(11510),scale(11542),
		scale(11573),scale(11604),scale(11636),scale(11667),
		scale(11698),scale(11730),scale(11761),scale(11792),
		scale(11823),scale(11855),scale(11886),scale(11917),
		scale(11948),scale(11980),scale(12011),scale(12042),
		scale(12073),scale(12104),scale(12135),scale(12166),
		scale(12198),scale(12229),scale(12260),scale(12291),
		scale(12322),scale(12353),scale(12384),scale(12415),
		scale(12446),scale(12477),scale(12508),scale(12539),
		scale(12570),scale(12601),scale(12632),scale(12663),
		scale(12694),scale(12724),scale(12755),scale(12786),
		scale(12817),scale(12848),scale(12879),scale(12909),
		scale(12940),scale(12971),scale(13002),scale(13033),
		scale(13063),scale(13094),scale(13125),scale(13155),
		scale(13186),scale(13217),scale(13247),scale(13278),
		scale(13309),scale(13339),scale(13370),scale(13400),
		scale(13431),scale(13462),scale(13492),scale(13523),
		scale(13553),scale(13584),scale(13614),scale(13645),
		scale(13675),scale(13706),scale(13736),scale(13766),
		scale(13797),scale(13827),scale(13858),scale(13888),
		scale(13918),scale(13949),scale(13979),scale(14009),
		scale(14039),scale(14070),scale(14100),scale(14130),
		scale(14160),scale(14191),scale(14221),scale(14251),
		scale(14281),scale(14311),scale(14342),scale(14372),
		scale(14402),scale(14432),scale(14462),scale(14492),
		scale(14522),scale(14552),scale(14582),scale(14612),
		scale(14642),scale(14672),scale(14702),scale(14732),
		scale(14762),scale(14792),scale(14822),scale(14852),
		scale(14881),scale(14911),scale(14941),scale(14971),
		scale(15001),scale(15030),scale(15060),scale(15090),
		scale(15120),scale(15149),scale(15179),scale(15209),
		scale(15238),scale(15268),scale(15298),scale(15327),
		scale(15357),scale(15387),scale(15416),scale(15446),
		scale(15475),scale(15505),scale(15534),scale(15564),
		scale(15593),scale(15623),scale(15652),scale(15682),
		scale(15711),scale(15740),scale(15770),scale(15799),
		scale(15829),scale(15858),scale(15887),scale(15916),
		scale(15946),scale(15975),scale(16004),scale(16034),
		scale(16063),scale(16092),scale(16121),scale(16150),
		scale(16179),scale(16209),scale(16238),scale(16267),
		scale(16296),scale(16325),scale(16354),scale(16383),
		scale(16412),scale(16441),scale(16470),scale(16499),
		scale(16528),scale(16557),scale(16586),scale(16615),
		scale(16643),scale(16672),scale(16701),scale(16730),
		scale(16759),scale(16788),scale(16816),scale(16845),
		scale(16874),scale(16903),scale(16931),scale(16960),
		scale(16989),scale(17017),scale(17046),scale(17074),
		scale(17103),scale(17132),scale(17160),scale(17189),
		scale(17217),scale(17246),scale(17274),scale(17303),
		scale(17331),scale(17360),scale(17388),scale(17416),
		scale(17445),scale(17473),scale(17501),scale(17530),
		scale(17558),scale(17586),scale(17615),scale(17643),
		scale(17671),scale(17699),scale(17727),scale(17756),
		scale(17784),scale(17812),scale(17840),scale(17868),
		scale(17896),scale(17924),scale(17952),scale(17980),
		scale(18008),scale(18036),scale(18064),scale(18092),
		scale(18120),scale(18148),scale(18176),scale(18204),
		scale(18232),scale(18260),scale(18287),scale(18315),
		scale(18343),scale(18371),scale(18398),scale(18426),
		scale(18454),scale(18482),scale(18509),scale(18537),
		scale(18564),scale(18592),scale(18620),scale(18647),
		scale(18675),scale(18702),scale(18730),scale(18757),
		scale(18785),scale(18812),scale(18840),scale(18867),
		scale(18894),scale(18922),scale(18949),scale(18976),
		scale(19004),scale(19031),scale(19058),scale(19086),
		scale(19113),scale(19140),scale(19167),scale(19194),
		scale(19221),scale(19249),scale(19276),scale(19303),
		scale(19330),scale(19357),scale(19384),scale(19411),
		scale(19438),scale(19465),scale(19492),scale(19519),
		scale(19546),scale(19573),scale(19599),scale(19626),
		scale(19653),scale(19680),scale(19707),scale(19733),
		scale(19760),scale(19787),scale(19814),scale(19840),
		scale(19867),scale(19894),scale(19920),scale(19947),
		scale(19973),scale(20000),scale(20026),scale(20053),
		scale(20079),scale(20106),scale(20132),scale(20159),
		scale(20185),scale(20212),scale(20238),scale(20264),
		scale(20291),scale(20317),scale(20343),scale(20369),
		scale(20396),scale(20422),scale(20448),scale(20474),
		scale(20500),scale(20527),scale(20553),scale(20579),
		scale(20605),scale(20631),scale(20657),scale(20683),
		scale(20709),scale(20735),scale(20761),scale(20787),
		scale(20813),scale(20838),scale(20864),scale(20890),
		scale(20916),scale(20942),scale(20967),scale(20993),
		scale(21019),scale(21045),scale(21070),scale(21096),
		scale(21122),scale(21147),scale(21173),scale(21198),
		scale(21224),scale(21249),scale(21275),scale(21300),
		scale(21326),scale(21351),scale(21377),scale(21402),
		scale(21427),scale(21453),scale(21478),scale(21503),
		scale(21529),scale(21554),scale(21579),scale(21604),
		scale(21629),scale(21655),scale(21680),scale(21705),
		scale(21730),scale(21755),scale(21780),scale(21805),
		scale(21830),scale(21855),scale(21880),scale(21905),
		scale(21930),scale(21955),scale(21980),scale(22004),
		scale(22029),scale(22054),scale(22079),scale(22104),
		scale(22128),scale(22153),scale(22178),scale(22202),
		scale(22227),scale(22252),scale(22276),scale(22301),
		scale(22325),scale(22350),scale(22374),scale(22399),
		scale(22423),scale(22448),scale(22472),scale(22496),
		scale(22521),scale(22545),scale(22569),scale(22594),
		scale(22618),scale(22642),scale(22666),scale(22691),
		scale(22715),scale(22739),scale(22763),scale(22787),
		scale(22811),scale(22835),scale(22859),scale(22883),
		scale(22907),scale(22931),scale(22955),scale(22979),
		scale(23003),scale(23027),scale(23050),scale(23074),
		scale(23098),scale(23122),scale(23146),scale(23169),
		scale(23193),scale(23217),scale(23240),scale(23264),
		scale(23287),scale(23311),scale(23335),scale(23358),
		scale(23382),scale(23405),scale(23428),scale(23452),
		scale(23475),scale(23499),scale(23522),scale(23545),
		scale(23569),scale(23592),scale(23615),scale(23638),
		scale(23661),scale(23685),scale(23708),scale(23731),
		scale(23754),scale(23777),scale(23800),scale(23823),
		scale(23846),scale(23869),scale(23892),scale(23915),
		scale(23938),scale(23961),scale(23984),scale(24006),
		scale(24029),scale(24052),scale(24075),scale(24097),
		scale(24120),scale(24143),scale(24165),scale(24188),
		scale(24211),scale(24233),scale(24256),scale(24278),
		scale(24301),scale(24323),scale(24346),scale(24368),
		scale(24390),scale(24413),scale(24435),scale(24457),
		scale(24480),scale(24502),scale(24524),scale(24546),
		scale(24569),scale(24591),scale(24613),scale(24635),
		scale(24657),scale(24679),scale(24701),scale(24723),
		scale(24745),scale(24767),scale(24789),scale(24811),
		scale(24833),scale(24855),scale(24876),scale(24898),
		scale(24920),scale(24942),scale(24964),scale(24985),
		scale(25007),scale(25029),scale(25050),scale(25072),
		scale(25093),scale(25115),scale(25136),scale(25158),
		scale(25179),scale(25201),scale(25222),scale(25243),
		scale(25265),scale(25286),scale(25307),scale(25329),
		scale(25350),scale(25371),scale(25392),scale(25414),
		scale(25435),scale(25456),scale(25477),scale(25498),
		scale(25519),scale(25540),scale(25561),scale(25582),
		scale(25603),scale(25624),scale(25645),scale(25665),
		scale(25686),scale(25707),scale(25728),scale(25749),
		scale(25769),scale(25790),scale(25811),scale(25831),
		scale(25852),scale(25872),scale(25893),scale(25913),
		scale(25934),scale(25954),scale(25975),scale(25995),
		scale(26016),scale(26036),scale(26056),scale(26077),
		scale(26097),scale(26117),scale(26137),scale(26158),
		scale(26178),scale(26198),scale(26218),scale(26238),
		scale(26258),scale(26278),scale(26298),scale(26318),
		scale(26338),scale(26358),scale(26378),scale(26398),
		scale(26418),scale(26437),scale(26457),scale(26477),
		scale(26497),scale(26516),scale(26536),scale(26556),
		scale(26575),scale(26595),scale(26614),scale(26634),
		scale(26654),scale(26673),scale(26692),scale(26712),
		scale(26731),scale(26751),scale(26770),scale(26789),
		scale(26809),scale(26828),scale(26847),scale(26866),
		scale(26885),scale(26905),scale(26924),scale(26943),
		scale(26962),scale(26981),scale(27000),scale(27019),
		scale(27038),scale(27057),scale(27076),scale(27094),
		scale(27113),scale(27132),scale(27151),scale(27170),
		scale(27188),scale(27207),scale(27226),scale(27244),
		scale(27263),scale(27281),scale(27300),scale(27318),
		scale(27337),scale(27355),scale(27374),scale(27392),
		scale(27411),scale(27429),scale(27447),scale(27466),
		scale(27484),scale(27502),scale(27520),scale(27538),
		scale(27557),scale(27575),scale(27593),scale(27611),
		scale(27629),scale(27647),scale(27665),scale(27683),
		scale(27701),scale(27719),scale(27736),scale(27754),
		scale(27772),scale(27790),scale(27808),scale(27825),
		scale(27843),scale(27861),scale(27878),scale(27896),
		scale(27913),scale(27931),scale(27948),scale(27966),
		scale(27983),scale(28001),scale(28018),scale(28036),
		scale(28053),scale(28070),scale(28087),scale(28105),
		scale(28122),scale(28139),scale(28156),scale(28173),
		scale(28190),scale(28208),scale(28225),scale(28242),
		scale(28259),scale(28275),scale(28292),scale(28309),
		scale(28326),scale(28343),scale(28360),scale(28377)
};
