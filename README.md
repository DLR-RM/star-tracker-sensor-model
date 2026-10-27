# Star Tracker Sensor Model FMUs

This repository contains compiled Functional Mock-up Units (FMUs) for a
system-level star tracker sensor model. A BibTeX entry template for citing the
associated publication is provided in `CITATION.bib`; publication-specific
fields are left open until the paper is published.

The FMUs are included in `bin/` for Linux and Windows:

| File | Platform | FMI mode |
| --- | --- | --- |
| `bin/startracker_error_model_n16_clk0p1_v140_linux.fmu` | Linux 64 bit | Co-Simulation |
| `bin/startracker_error_model_n16_clk0p1_v140_win.fmu` | Windows 64 bit | Co-Simulation |

The exported model is intended for system-level, closed-loop simulation of
guidance, navigation, and control algorithms. It is not an image-level optical
star tracker simulator.

## Download
Please download the binaries from the latest release on the Github page (right side, under releases). 
There are known issues with FMUs that are synchronized via git.


## Model Overview

The model represents the attitude-relevant behaviour of a star tracker at vector
level. It uses catalogue star directions, the spacecraft position and attitude,
the sensor mounting, field-of-view geometry, directional measurement noise, and
analytical Sun, Earth, and Moon geometry to produce:

- an estimated spacecraft attitude quaternion,
- a geometry-dependent covariance estimate,
- visible catalogue star directions and identifiers,
- Boolean flags indicating Sun, Earth, or Moon intrusion into the configured
  avoidance region.

The model omits pixel-level image formation, optics, detector
physics, centroid extraction, and explicit star-identification failures. These
effects are represented only indirectly through directional disturbance of the
identified star vectors. Consequently, the FMU is most appropriate for
closed-loop prototyping where downstream algorithms consume attitude,
covariance, and availability information.

Internally, the model performs the following operations:

1. The true spacecraft attitude `qTrue` and position `rBody_eci` define the
   body frame with respect to the Earth-centred inertial frame.
2. The sensor mounting `measToBodyEuler` and line of sight `lineOfSight_meas`
   define the measurement frame and boresight.
3. Catalogue stars from the Bright Star Catalogue are selected if they lie
   within the configured field of view.
4. The selected star vectors are perturbed by an angular disturbance with mean
   `muDist` and standard deviation `sigmaDist`.
5. The perturbed body-frame vectors and the corresponding inertial catalogue
   vectors are used in a QUEST attitude-estimation step.
6. Analytical ephemerides are used to evaluate Sun, Earth, and Moon field-of-view
   intrusion flags.

The number of tracked stars and the model clock period are fixed in this FMU
export. This repository provides an export with:

| Quantity | Value |
| --- | --- |
| Maximum number of tracked stars | 16 |
| Model clock period | 0.1 s |
| Nominal update rate | 10 Hz |

The fixed star count is visible in the interface dimensions, for example
`visibleStarsECI[3,16]` and `visibleStarsID[16]`. Changing the tracked-star
limit or clock period requires a new FMU export.

## Coordinates and Conventions

The model uses an Earth-centred inertial frame (ECI), a spacecraft body frame,
and a star tracker measurement frame. The catalogue frame and ECI frame are
treated as aligned for the intended system-level simulations.

Quaternions use the ordering:

```text
{p1, p2, p3, p0}
```

where `p0` is the scalar component. Both `qTrue` and `qEst` represent the
rotation from the ECI frame into the body frame.

Angles are provided in radians unless otherwise stated. The Euler mounting
parameter `measToBodyEuler` follows an intrinsic x-y-z roll-pitch-yaw sequence
and describes the rotation from the body frame into the measurement frame.

## FMU Interface

### Inputs

| Variable | Size | Type | Unit | Description |
| --- | ---: | --- | --- | --- |
| `rBody_eci` | 3 | Real input | m | Position of the spacecraft body frame origin in ECI coordinates. |
| `qTrue` | 4 | Real input | - | True spacecraft attitude quaternion `{p1,p2,p3,p0}` rotating ECI into the body frame. |

### Outputs

| Variable | Size | Type | Unit | Description |
| --- | ---: | --- | --- | --- |
| `visibleStarsECI` | 3 x 16 | Real output, discrete | - | Unit direction vectors of the selected visible catalogue stars, resolved in ECI coordinates. Columns correspond to tracked stars. |
| `visibleStarsID` | 16 | Real output, discrete | - | Identifiers of the selected visible stars. These are HR numbers from the Bright Star Catalogue. |
| `qEst` | 4 | Real output, discrete | - | Estimated attitude quaternion `{p1,p2,p3,p0}` rotating ECI into the body frame. |
| `earthInFOV` | 1 | Boolean output, discrete | - | `true` if Earth lies inside the configured Earth avoidance region; the measurement should be treated as invalid. |
| `sunInFOV` | 1 | Boolean output, discrete | - | `true` if the Sun lies inside the configured Sun avoidance region; the measurement should be treated as invalid. |
| `moonInFOV` | 1 | Boolean output, discrete | - | `true` if the Moon lies inside the configured Moon avoidance region; the measurement may be invalid. |
| `covEst` | 3 x 3 | Real output, discrete | - | Estimated covariance matrix of the small attitude-error vector with respect to the body axes. |

The covariance estimate is geometry-dependent. It should be interpreted as a
relative indicator of attitude-observation quality. 

### Parameters

| Variable | Size | Default | Unit | Description |
| --- | ---: | ---: | --- | --- |
| `lineOfSight_meas` | 3 | `[0, 0, 1]` | - | Sensor line-of-sight vector in the measurement frame. The default corresponds to the positive measurement-frame z-axis. |
| `measToBodyEuler` | 3 | `[0, 0, 0]` | rad | Intrinsic x-y-z Euler angles defining the body-to-measurement mounting rotation. |
| `earthAvoidanceAngle` | 1 | `0.34906585039887` | rad | Minimum allowed angular separation between Earth's horizon and the sensor boresight. |
| `sunAvoidanceAngle` | 1 | `0.45378560551853` | rad | Minimum allowed angular separation between the Sun and the sensor boresight. |
| `MoonAvoidanceAngle` | 1 | `0` | rad | Minimum allowed angular separation between the Moon and the sensor boresight. |
| `muDist` | 1 | `0` | rad | Mean angular bias of the directional disturbance applied to visible star vectors. |
| `sigmaDist` | 1 | `1.7453292519943e-05` | rad | Standard deviation of the angular directional disturbance applied to visible star vectors. The default is 0.001 deg. |
| `FOVAngle` | 1 | `0.34906585039887` | rad | Full characteristic field-of-view angle. The default is 20 deg. |
| `year` | 1 | `2000` | - | Gregorian simulation start year for the analytical ephemeris model. |
| `month` | 1 | `1` | - | Gregorian simulation start month, 1 to 12. |
| `day` | 1 | `1` | - | Gregorian simulation start day, 1 to 31. |
| `hour` | 1 | `0` | - | Gregorian simulation start hour, 0 to 23. |
| `minute` | 1 | `0.0` | min | Gregorian simulation start minute, including fractional minutes for seconds. |
| `localSeed` | 1 | `13300988` | - | Local seed for random-number generation. |
| `globalSeed` | 1 | `39941994` | - | Global seed for random-number generation. |

## Notes on Visibility and Validity

The FMU returns up to 16 visible catalogue stars per update. If more than 16
stars satisfy the FOV criterion, the retained subset is determined by the
catalogue search order. The model does not sort visible stars by brightness,
expected centroid quality, or geometric conditioning.

The `earthInFOV`, `sunInFOV`, and `moonInFOV` flags indicate geometric intrusion
into the corresponding avoidance region. The FMU does not automatically suppress
or replace `qEst` when a flag is true. Downstream simulation code should decide
how to handle flagged measurements, for example by rejecting the attitude update
or increasing measurement uncertainty.

## Limitations

The FMU is a system-level abstraction. It does not model optical image
formation, detector noise, saturation, point-spread functions, centroiding,
motion blur, catalogue-matching failures, or hardware-specific processing.
Directional noise and covariance are not calibrated to a particular flight
sensor in this repository. Quantitative validation against image-level
simulation or hardware measurements is outside the scope of the provided FMUs.

The model is therefore suitable for reproducible closed-loop prototyping and
algorithm studies, but it should not be interpreted as a flight-qualified or
sensor-vendor-specific performance model.

## License

See `LICENSE` for repository licensing information. Each FMU also contains
third-party license files in its `documentation/` directory.

## Contact
Tobias Kamp [@tkmamp](https://github.com/tkmamp) 

tobias.kamp@dlr.de

German Aerospace Center
Deutsches Zentrum für Luft und Raumfahrt e.V. (DLR)
 
Institute of Robotics and Mechatronics

Münchener Straße 20

D-82234 Wessling, Germany