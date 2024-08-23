// 4th order piecewise approximation table, requires 6*6*4 = 144 bytes.
// - Values for R0=100Ω
// - maximum error: +- 0.0001°C
// - valid range: [-200°C, 900°C]
// - no need for binary search
#[rustfmt::skip]
const LOOKUP_TABLE_R100_RESISTANCES: [f32; 6] = [
    // maximum valid resistance for the params at the same index
    61.735252380371094,
    103.28826141357422,
    215.15463256835938,
    303.1486511230469,
    377.0411682128906,
    404.9695129394531,
];

#[rustfmt::skip]
const LOOKUP_TABLE_R100_PARAMS: [[f32; 5]; 6] = [
    // polynomial params [a, b, c, d, e])
    [1.905779032269027e-09,  -7.117464772295086e-06,  0.002669762995725334,  2.2213402029534284, -242.01002426981225 ],
    [3.361247358952037e-08,  -1.4788341478359002e-05, 0.003384182701259624,  2.1910160607212203, -241.51631297597694 ],
    [9.66907965148285e-10,    3.129280635414738e-07,  0.0008169962519715037, 2.3819731827926165, -246.7767415448826  ],
    [1.8234807703590488e-09, -4.2884754332702424e-07, 0.0010606402255259881, 2.346023738724996,  -244.7681606750746  ],
    [3.3744564807077837e-09, -2.322593539935175e-06,  0.0019310253713338127, 2.1675767346068144, -231.00034632603806 ],
    [5.344514925701199e-09,  -5.238631218437194e-06,  0.003551305474814012,  1.767021807936818,  -193.8273692877731  ],
];

/// Inverse Callendar-Van Dusen equation for R0=100Ω,
/// based on piecewise reconstruction with 4th order polynomials.
/// The maximum error is ±1e-5°C over the whole temperature range [-200°C, 900°C] with f64, and ±1e-4°C for f32.
/// but degrades rapidly beyond that range. Inputs that correspond to values outside
/// of this range will yield unpredictable outcomes.
pub fn resistance_to_temperature_r100(r: f32) -> f32 {
    // Find the valid parameter index
    let param_index = LOOKUP_TABLE_R100_RESISTANCES
        .iter()
        .position(|&max_valid_resistance| r < max_valid_resistance)
        .unwrap_or(LOOKUP_TABLE_R100_RESISTANCES.len() - 1);
    let [a, b, c, d, e] = LOOKUP_TABLE_R100_PARAMS[param_index];
    let r2 = r * r;
    let r3 = r2 * r;
    let r4 = r2 * r2;
    a * r4 + b * r3 + c * r2 + d * r + e
}

/// Callendar-Van Dusen equation for R0=100Ω
pub fn temperature_to_resistance_r100(t: f32) -> f32 {
    const R0: f32 = 100.0;
    const A: f32 = 3.9083e-3;
    const B: f32 = -5.775e-7;
    const C: f32 = -4.183e-12;
    if t < 0.0 {
        let t2 = t * t;
        let t3 = t2 * t;
        R0 * (1.0 + A * t + B * t2 + C * (t - 100.0) * t3)
    } else {
        let t2 = t * t;
        R0 * (1.0 + A * t + B * t2)
    }
}

#[cfg(test)]
mod test {
    use super::resistance_to_temperature_r100;
    use approx::assert_abs_diff_eq;

    #[test]
    #[rustfmt::skip]
    fn test_lookup_table_r100() {
        assert_abs_diff_eq!(resistance_to_temperature_r100(18.52008056640625), -200.0, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(23.206966400146484), -189.1099853515625, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(27.85323143005371), -178.21995544433594, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(32.462501525878906), -167.32772827148438, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(37.03547668457031), -156.43771362304688, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(41.57643508911133), -145.5454864501953, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(46.0858154296875), -134.6554718017578, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(50.567596435546875), -123.76324462890625, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(55.02195358276367), -112.87322235107422, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(59.45254898071289), -101.98100280761719, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(63.859310150146484), -91.09098052978516, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(68.24559783935547), -80.19876098632812, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(72.61107635498047), -69.3087387084961, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(76.95879364013672), -58.4165153503418, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(81.28814697265625), -47.52649688720703, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(85.60191345214844), -36.634273529052734, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(89.89920043945312), -25.744251251220703, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(94.18248748779297), -14.852029800415039, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(98.45062255859375), -3.962007999420166, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(102.70576477050781), 6.930213928222656, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(106.94634246826172), 17.820236206054688, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(111.17407989501953), 28.71245765686035, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(115.38726043701172), 39.60247802734375, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(119.58760070800781), 50.49470138549805, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(123.77338409423828), 61.38472366333008, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(127.9463119506836), 72.27694702148438, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(132.10470581054688), 83.1669692993164, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(136.25022888183594), 94.05918884277344, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(140.3812255859375), 104.94921112060547, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(144.4993438720703), 115.8414306640625, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(148.60293579101562), 126.73145294189453, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(152.69366455078125), 137.62367248535156, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(156.7698516845703), 148.51370239257812, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(160.8331756591797), 159.40591430664062, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(164.88197326660156), 170.2959442138672, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(168.91787719726562), 181.1881561279297, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(172.9392852783203), 192.07818603515625, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(176.9477996826172), 202.9704132080078, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(180.94180297851562), 213.8604278564453, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(184.92291259765625), 224.75265502929688, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(188.88951110839844), 235.64266967773438, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(192.8432159423828), 246.53489685058594, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(196.7824249267578), 257.4249267578125, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(200.70872497558594), 268.317138671875, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(204.6205291748047), 279.2071533203125, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(208.51942443847656), 290.099365234375, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(212.4038543701172), 300.9894104003906, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(216.2753448486328), 311.8816223144531, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(220.13235473632812), 322.7716369628906, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(223.97645568847656), 333.66387939453125, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(227.8060760498047), 344.55389404296875, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(231.6227569580078), 355.44610595703125, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(235.4249725341797), 366.33612060546875, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(239.21426391601562), 377.2283630371094, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(242.9890899658203), 388.1183776855469, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(246.75096130371094), 399.0105895996094, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(250.49839782714844), 409.900634765625, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(254.2328643798828), 420.7928466796875, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(257.952880859375), 431.682861328125, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(261.65997314453125), 442.5750732421875, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(265.35260009765625), 453.4651184082031, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(269.03228759765625), 464.3573303222656, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(272.697509765625), 475.2473449707031, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(276.34979248046875), 486.13958740234375, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(279.98760986328125), 497.02960205078125, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(283.61248779296875), 507.92181396484375, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(287.222900390625), 518.8118286132812, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(290.82037353515625), 529.7040405273438, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(294.4034118652344), 540.5940551757812, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(297.9734802246094), 551.486328125, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(301.52911376953125), 562.3763427734375, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(305.07177734375), 573.2685546875, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(308.6000061035156), 584.1585693359375, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(312.1152648925781), 595.05078125, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(315.6161193847656), 605.9407958984375, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(319.10394287109375), 616.8330078125, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(322.5774230957031), 627.7230834960938, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(326.0378723144531), 638.6152954101562, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(329.4839172363281), 649.5053100585938, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(332.9169616699219), 660.3975219726562, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(336.3356018066406), 671.2875366210938, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(339.7412414550781), 682.1797485351562, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(343.1324768066406), 693.0697631835938, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(346.5107421875), 703.9620361328125, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(349.8746032714844), 714.85205078125, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(353.2254333496094), 725.7442626953125, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(356.5618896484375), 736.63427734375, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(359.88531494140625), 747.5264892578125, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(363.1943664550781), 758.41650390625, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(366.4903869628906), 769.3087158203125, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(369.7720642089844), 780.19873046875, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(373.04071044921875), 791.0910034179688, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(376.29498291015625), 801.9810180664062, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(379.53619384765625), 812.8732299804688, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(382.7630615234375), 823.7632446289062, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(385.97686767578125), 834.6554565429688, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(389.17633056640625), 845.5454711914062, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(392.3627624511719), 856.4376831054688, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(395.53485107421875), 867.3277587890625, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(398.69384765625), 878.219970703125, epsilon = 0.0001);
    }
}
