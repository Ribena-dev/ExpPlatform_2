# Final UROP report
**Student:** Devinaa Kumeresh  
**Student ID:** A0266490X  
**Semester:** Sem 2 AY24/25 Sem 1 Ay25/26  
*Word Count:~4500*

## Acknowledgments
I want to express thanks to everyone who has helped or guided me during this project 

I want to specifically thank the following individuals: 
My supervisor, Dr. Yen Shih Cheng, for his invaluable guidance and support throughout this project. 

Mr. Herikstad Roger and Lim Wee Chiek (Clement) for their technical expertise and assistance.


## Table of Contents
1. [Introduction and Background](#introduction-and-background)
2. [Project Objectives](#project-objectives)
3. [System Architecture](#system-architecture)
4. [Implementation](#implementation)
5. [Testing Environment and Constraints](#testing-environment-and-constraints)
6. [Challenges](#challenges)
7. [Future Work](#future-work)
8. [Conclusions](#conclusions)
9. [Links](#Links)

---

## Introduction and Background

### Background

Understanding how the brain performs computation has been identified as one of the Grand Challenges for Engineering in the 21st century. This project investigates place cell activity in animals, building upon foundational work recognized with the 2014 Nobel Prize in Physiology or Medicine for the discovery of cells that constitute a positioning system in the brain.

The experimental approach involves creating both virtual and physical navigation environments. Animals are trained to navigate through virtual reality while neural activity and eye movements are recorded. Subsequently, a physical experimental arena using a robotic mobile platform enables tracking of location and eye movements during navigation. This dual-environment paradigm allows direct comparison of neural activity patterns between virtual and physical spatial navigation.

### Experimental Protocol

The experimental workflow follows a structured sequence designed to study spatial navigation behavior in a controlled environment:

```
┌─────────────────────────┐
│  Map Experimental Area  │ (SLAM mapping without animal present)
│   (Without Subject)     │
└───────────┬─────────────┘
            │
            ▼
┌─────────────────────────┐
│   Register Poster       │ (Mark virtual map locations corresponding
│      Locations          │  to physical poster positions)
└───────────┬─────────────┘
            │
            ▼
┌─────────────────────────┐
│  Configure Parameters   │ (Set trial structure, reward contingencies
│      via GUI            │  via GUI interface)
└───────────┬─────────────┘
            │
            ▼
┌─────────────────────────┐
│  Generate Randomized    │ (Create poster presentation sequence
│   Poster Sequence       │  with constraints)
└───────────┬─────────────┘
            │
            ▼
┌─────────────────────────┐
│  iPad Displays Target   │
│       Poster            │
└───────────┬─────────────┘
            │
            ▼
┌─────────────────────────┐
│   Animal Navigates      │
│  Platform to Target     │
└───────────┬─────────────┘
            │
      ┌─────┴─────┐
      │           │
      ▼           ▼
 Successful   Unsuccessful
 Arrival      Navigation
      │           │
      ▼           ▼
┌──────────┐  ┌──────────┐
│ Dispense │  │    No    │
│  Reward  │  │  Reward  │
└────┬─────┘  └────┬─────┘
      │           │
      └─────┬─────┘
            │
            ▼
┌─────────────────────────┐
│   Present New Poster    │
│        Target           │
└───────────┬─────────────┘
            │
            ▼
┌─────────────────────────┐
│    Log Experimental     │ (Record trajectory, timing,
│         Data            │  behavioral responses)
└─────────────────────────┘
```

**Table 1: Experimental Protocol Phase Details**

| Phase | Component | Description | Output/Data |
|-------|-----------|-------------|-------------|
| **1. Mapping** | SLAM System | Designated area mapped using gmapping without animal present | Occupancy grid map (.pgm, .yaml) |
| **2. Registration** | Map Editor | Poster locations marked on virtual map to correspond with physical positions | Waypoint coordinates |
| **3. Configuration** | gui.py | Experimental parameters entered including trial count, reward timing, delays | Experiment config file (.pkl) |
| **4. Sequence Generation** | poster_distribution.py | Random poster sequence generated with paired exclusion constraints | Image ID queue |
| **5. Stimulus Display** | Flutter App / HTML | Target poster image displayed on iPad via TCP connection | Visual stimulus presented |
| **6. Navigation** | move_base + AMCL | Platform autonomously navigates to target poster location | Position trajectory |
| **7. Reward Delivery** | Solenoid Control | Upon successful arrival at target, liquid reward automatically dispensed | Reward timestamp, volume |
| **8. Data Logging** | Experiment_Sam.py | All trial events, timings, and outcomes recorded to file | Experimental log file |



---

## Project Objectives

### Continuation of Previous Work

This project continues previous development efforts on the Pioneer 3-AT platform. A significant hardware modification was the addition of a large rear-mounted shelf to accommodate experimental equipment including iPad display mount, juice delivery system, and electronics. This structural addition created challenges for obstacle avoidance as the extended rear profile was not covered by the original sensor configuration.

**Figure 1: [INSERT IMAGE - Pioneer 3-AT with rear shelf showing extended profile]**

### Core Objectives

The primary objectives for this iteration of development were organized into three main categories addressing obstacle avoidance, experimental validation, and system integration.

The first objective focused on enhanced obstacle avoidance capabilities. This involved extending detection coverage to include the large back shelf structure that had been added to the platform. The system needed to implement smooth deceleration behaviors as the robot approached obstacles, replacing the jerky motion that had characterized earlier turning maneuvers. Achieving fluid motion during navigation was considered essential for both equipment protection and subject comfort during experiments.

The second major objective centered on end-to-end experimental validation. This comprehensive testing aimed to verify the complete experimental workflow from initial setup through data collection. Particular attention was paid to validating the visual stimulus display interface application across target platforms, ensuring the system could produce high-quality maps with accurate localization, and confirming that the automated reward delivery system operated reliably under experimental conditions. Perhaps most critically, this validation needed to demonstrate proper synchronization between stimulus presentation, autonomous navigation, and reward delivery timing.

The third objective addressed system integration concerns. All hardware and software components needed to function together as a cohesive whole rather than as isolated subsystems. This required developing user-friendly interfaces that would allow researchers to configure experimental parameters without deep technical knowledge of the underlying systems. Comprehensive documentation of system operation was also essential to enable future researchers to maintain and extend the platform's capabilities.


Due to time constraints  eye tracking integration was not accomplished during this phase. While the platform includes provisions for mounting eye tracking hardware, the software integration required to synchronize eye movement data with experimental events remains untested. 
---

## System Architecture

### Hardware Components

The complete hardware system comprises multiple integrated subsystems supporting autonomous navigation, obstacle detection, stimulus presentation, and reward delivery.

**Table 2: Hardware Component Specifications**

| Category | Component | Specifications | Purpose |
|----------|-----------|----------------|---------|
| **Base Platform** | Pioneer 3-AT (P3AT) | Four-wheel drive mobile robot | Primary mobility platform |
| | OS | Ubuntu 16.04 | ROS computation and control |
| | Main Power | 24V DC battery | Robot motors and systems |
| | Power Regulation | 12V and 5V regulators | Sensor and peripheral power |
| **Primary Sensors** | Front LiDAR | 270° field of view | Navigation, obstacle detection, SLAM |
| | Rear LiDAR | 270° field of view, mounted on shelf edge | Back shelf collision prevention |
| | Front Sonar Array | 8 ultrasonic sensors | Supplementary forward detection |
| | Rear Sonar Array | 8 ultrasonic sensors | Supplementary rear detection |
| **Custom Equipment** | Equipment Shelf | Rear-mounted aluminum frame | Houses display and electronics |
| | iPad Mount | Articulated mounting arm | Visual stimulus presentation |
| | Juice Delivery | Solenoid valve (12V), reservoir | Automated liquid reward dispensing |
| | GPIO Controller | MCP2221 USB-to-GPIO | Hardware interface control |
| **Control Systems** | Manual Joystick | Wireless controller | Manual override capability |
| | Emergency Stops | Multiple e-stop buttons | Safety system |

**Power Distribution**: The secondary LiDAR connects to the robot's user power terminal via brown wire for 12V power and blue wire for ground connection. The solenoid valve for juice delivery similarly draws 12V power from the regulated supply, while the MCP2221 GPIO controller operates from USB bus power provided by the onboard computer.

**Figure 2: [INSERT IMAGE - Electrical system schematic showing power distribution]**

### Software Architecture

The software system is built on ROS Kinetic running on Ubuntu 16.04, utilizing Python 2.7 for ROS component integration. This middleware framework provides the publish-subscribe communication model that enables modular development and system integration.

**Table 3: Core Software Components**

| Component | Function |
|-----------|----------|
| RosAria | Robot motor interface and odometry |
| AMCL | Adaptive Monte Carlo localization |
| move_base | Dynamic path planning and execution |
| gmapping | SLAM mapping during environment exploration |
| sonar_get_dist.py | Sonar distance processing and array management |
| lidar_get_dist.py | LiDAR distance processing for front and rear units |
| combined_joystick.py | Obstacle avoidance logic and motion control |
| gui.py | Tkinter-based experiment configuration interface |
| Experiment_Sam.py | Main experimental controller and state machine |
| tcp_server.py | Display communication server (port 8000) |
| poster_distribution.py | Stimulus sequence generation with constraints |

The data processing pipeline follows a hierarchical structure. Sensors continuously publish raw data to ROS topics, with the front LiDAR publishing to /scan and the rear LiDAR to /back_scan, while sonar sensors aggregate their readings in /RosAria/sonar. Distance processing scripts subscribe to these sensor topics and compute minimum distances per array, publishing summarized data to /sonar_min and /lidar_distances topics. The combined_joystick.py node serves as the central motion arbiter, subscribing to both the processed distance data and joystick commands. This node implements the obstacle avoidance logic by checking clearances in the direction of intended motion and applying moving average filtering to smooth velocity commands. F

The experimental control flow operates independently of the navigation system but coordinates with it through ROS messaging. The gui.py interface allows researchers to configure experimental parameters including poster sequences, reward timing, and trial structure. Upon experiment launch, Experiment_Sam.py takes control, generating the poster distribution sequence and establishing TCP connection with the display device. As trials progress, this controller sends image identifiers to tcp_server.py running on port 8000, which relays them to the Flutter application for display. Simultaneously, the controller monitors the robot's position through AMCL pose estimates, triggering the juice dispensing system via MCP2221 GPIO commands when the platform successfully reaches target locations.

**Figure 3: [INSERT IMAGE - Software architecture flowchart showing data flow between components]**

---

## Implementation

### Simulation Attempts (Week 9)

Establishing a Gazebo simulation environment was attempted to enable safer testing and faster iteration. The Pioneer 3-AT URDF model was successfully constructed and validated. However, critical obstacles prevented deployment of a functional simulation environment.

The primary blocker was the absence of required Gazebo plugin libraries from the ROS Kinetic distribution. These sensor simulation libraries are essential for generating ROS topic data from simulated sensors. Additionally, ROS Kinetic (released 2016) and Ubuntu 16.04 are no longer actively maintained, with many package repositories having gone offline. Attempts to compile Gazebo 7 from source encountered numerous dependency conflicts that proved difficult to resolve. The documentation situation was complicated by defunct URLs, requiring use of the Internet Archive Wayback Machine to access historical installation instructions.

The decision was made to prioritize physical platform testing rather than investing substantial time in simulation environment setup with minimal success guarantee.

### Navigation System

#### Mapping and Localization

Maps were created using ROS gmapping SLAM, with AMCL for robot localization. Initial challenges included map rotation issues (90° offset) upon startup, resolved through TF frame alignment verification and improved mapping procedures. Maps were successfully created for the experiment room, maze room (MD2), and corridor environments.

**Figure 4: [INSERT IMAGE - Generated occupancy grid maps]**

Path planning used ROS move_base with Dynamic Window Approach for local planning. Poster locations were configured as navigation waypoints. Performance was optimized by tuning velocity gains and configuring reward zone sizes to reduce corner entrapment.

### Obstacle Avoidance

#### Initial Sonar-Only Limitations (Week 2)

The platform's 16 ultrasonic sonar sensors (8 front, 8 rear) theoretically provided 360° coverage but revealed significant limitations during initial testing. Several sensors provided inconsistent readings or experienced complete failures, compromising the reliability of the obstacle detection system. Angular gaps between sensor beams allowed objects positioned at specific angles to pass undetected, creating blind spots in the supposedly comprehensive coverage pattern. The continuous ultrasonic emissions at approximately 40 kHz created substantial auditory disturbance, which proved particularly problematic for animal experiments where acoustic comfort is essential. Performance also degraded significantly with certain surface materials and obstacle orientations, adding environmental sensitivity to the list of concerns.

**Figure 5: [INSERT IMAGE - Sonar array configuration showing coverage gaps]**

#### Dual-LiDAR Solution (Weeks 2-3)

A secondary LiDAR was integrated at the rear shelf edge to provide reliable obstacle detection. Advantages over sonar included: superior range accuracy (±2-3 cm vs ±5-10 cm), higher angular resolution (0.5° vs 30-40° per sensor), silent operation, and greater reliability. The LiDAR was mounted elevated to prevent shelf structure obstruction and connected to the 12V user power terminal. Point cloud visualization in RViz enabled configuration optimization.

**Figure 6: [INSERT IMAGE - Rear LiDAR mounting and coverage visualization]**

The final hybrid approach uses LiDAR as primary detection with sonar providing supplementary close-range coverage.

#### Sensor Processing (Weeks 7-8)

Initial implementation with large sonar arrays caused diagonal detection bugs affecting turning behavior. This was resolved by splitting into smaller functional arrays (back_left and back_right), enabling direction-specific clearance checking. The topic /sonar_min published minimum distances per array to combined_joystick.py for motion decisions.

#### Smooth Motion Control (Weeks 10-11)

Moving average velocity filtering was integrated to address jerky motion during obstacle avoidance. Velocity commands are averaged over a sliding window, producing gradual acceleration and deceleration. Additionally, velocity scaling based on obstacle proximity implements progressive speed reduction as clearance decreases, enabling smooth approaches to obstacles while maintaining safety.

#### Debugging (Week 12)

ROS bag recording enabled offline debugging when /sonar_min subscription issues were discovered. Recorded sensor data was replayed as simulation data to test detection algorithms and moving average functions in controlled conditions.

### Visual Stimulus Display

#### Flutter Application Development (Weeks 10-11)

A Flutter cross-platform application was developed for stimulus presentation. The architecture connects gui.py → Experiment_Sam.py → tcp_server (port 8000) → Flutter app displaying images.

Testing followed a systematic progression beginning with a simple TCP connection test app to verify basic connectivity. This was followed by simulated GUI testing on a Linux development machine to validate the experimental control flow. Platform integration testing then confirmed real-time image updates functioned correctly when connected to the actual robot. Finally, validation on Linux and Android platforms demonstrated cross-platform compatibility.

Features implemented included asset-based image storage for the animal stimuli, TCP/IP communication for receiving display commands, automatic image sequence handling, connection status monitoring, dynamic IP address and port configuration, and visual logging for debugging purposes.

**Figure 7: [INSERT IMAGE - Flutter app interface screenshots]**

#### iOS Deployment Challenges (Weeks 12-13)

Building for iOS 16 using macOS 15 and Xcode 16 was successful, but Apple Developer Program limitations were discovered. Free Apple Developer Accounts only allow apps to remain installed for 7 days before expiration, requiring weekly reinstallation. A paid account costing $99 USD per year provides extended validity and proper distribution capabilities.

#### HTML/JavaScript Fallback (Week 12)

An interim web-based solution (index.html) was developed as backup. It displays embedded base64-encoded animal images (pig, rabbit, crocodile, donkey, cat, camel) according to received numeric codes, connects to HTTP server (python-server.py) for updates, and operates in iPad Edge browser. While functional, limitations include manual file transfer requirements and browser dependency.

### Poster Distribution Algorithm (Weeks 4-5)

A paired exclusion system prevents immediate stimulus repetition. Poster pairs are randomly selected, with the chosen pair blocked from the next selection round then unblocked the following round. This continues until the required queue length is achieved. Analysis of generated sequences (Week 7) revealed some repetitive patterns despite randomization, suggesting weighted sequence generation could improve variation.

### Reward Delivery System

#### Hardware (Weeks 4-8)

The circuit architecture includes: MCP2221 GPIO controller → 7400-series IC → MOSFET → Solenoid valve (12V). A manual trigger button provides emergency/testing activation.

**Component Selection**: Initial attempts using PyUSB library encountered Python 2.7 compatibility issues and low-level USB communication instability. The solution migrated to MCP2221 with C library (libmcp2221 from github.com/ZakKemble/libmcp2221.git), providing robust hardware interfacing compatible with ROS Python 2.7 environment.

**Figure 8: [INSERT IMAGE - Juice dispenser circuit assembly]**

#### Software Integration (Week 8)

Simple GPIO control scripts were developed to activate the solenoid for specified durations. Features include: controlled duration/amount specification, integration with gui.py for parameter configuration, and variable delay options (consistent or inconsistent intervals).

All hardware components were purchased and assembled by Week 8. Testing confirmed reliable dispensing with linear duration-volume relationship.

---

## Testing Environment and Constraints

### Testing Environment Challenges

The primary experimental environment, referred to as the operations room, presented significant logistical challenges that fundamentally shaped the development and testing approach throughout the project duration.

The operations room itself is characterized by a smaller rectangular layout compared to the more spacious development areas available elsewhere in the facility. This confined geometry creates limited maneuvering space for initial testing and debugging activities, presenting higher collision risk during the algorithm development phase when system behavior may be unpredictable. The tight spatial tolerances demanded greater navigation accuracy than would be required in more open environments, making this space inappropriate for early-stage testing where failures and collisions are expected as part of the learning process.

Beyond spatial constraints, the room serves as active housing for monkey subjects used in various behavioral experiments. The presence of animals in the space creates scheduling conflicts between ongoing behavioral research sessions and robotic platform development work. Testing cannot occur when animals are present in the room, restricting access to narrow windows between experimental sessions. This limitation significantly reduced the total time available for validation testing in the actual target environment.

Biosafety requirements imposed additional barriers to frequent testing iterations. The platform must undergo complete sanitization before entering the monkey room to prevent potential pathogen introduction that could compromise animal health or experimental results. Similarly, post-testing decontamination is required before the platform can be removed from the space. These cleaning protocols are time-consuming and involve chemical sanitization agents that pose potential risks to electronic components and mechanical systems. The necessity of repeated sanitization cycles for each testing session substantially delays development iteration cycles, making it impractical to conduct the frequent small tests that characterize agile development methodologies.

### Adapted Testing Strategy

Given these constraints, most development work happened in alternative spaces where access was unrestricted. The MD2 maze room became the primary testing ground for obstacle avoidance development, since its large open layout meant algorithm failures could be tolerated without equipment damage. Corridor spaces were used for straight-line navigation and velocity tuning, while laboratory areas served as integration testing environments where complete experimental workflows could be run through without the operations room restrictions.
Testing only moved to the operations room after core functionality was validated elsewhere. These sessions were brief and focused on the most critical checks: verifying that maps of the actual space were good enough, confirming the robot could navigate to poster locations in the tighter confines, and demonstrating that the complete workflow functioned correctly. This tiered approach worked well, delivering a functional system despite limited access to the target environment.

---

## Challenges

### iOS Application Deployment

Building for iOS proved complex due to macOS/Xcode/iOS SDK compatibility issues. Initial compilation attempts failed, and GitHub Actions cloud builds succeeded but couldn't transfer to iPad. The free Apple Developer Account limitation (7-day app expiration) was discovered late in development.

**Solution**: HTML/JavaScript web app provided immediate deployment capability while Flutter remains ready for institutional developer account deployment.

### Flutter TCP Connection Issues 

Initial attempts resulted in infinite reconnection loops without errors. Debugging revealed IP address mismatch between configuration and actual platform address. Resolution included implementing dynamic IP configuration and enhanced error logging.

### Sonar Array Debugging

Diagonal detection bugs caused incorrect motion blocking during turning. Root cause was treating all sensors in a single large array rather than direction-specific arrays. Solution split sonar sensors into back_left and back_right arrays with separate minimum distance calculations per direction.

discovered /sonar_min subscription timing issues where echoing the topic showed correct data but subscription callbacks received zeros. ROS bag replay enabled offline investigation.

### Misallanous issues

**Wiring Problems**: Loose front panel wire connection caused intermittent failures.

**USB Recognition**: USB devices not recognized after connection.
 By remove all USB devices, connect one at a time, wait for recognition, restart platform before connecting next device, it seems to resolve issues

**SSH Performance**: Persistent slow SSH connections caused delays launching gui.py and fixation task windows. Partially mitigated but remains ongoing issue.

### Navigation Tuning 

Robot frequently stuck in corners during navigation. Solutions implemented: increased reward zone size, reduced angular velocity gain, tuned linear velocity for smoother approach.

---

## Future Work

### High Priority

**Virtual Boundary System**: For the rectangular operations room, implement dual-rectangle boundaries with an outer rectangle (physical walls detected by sensors) and inner rectangle (virtual boundary preventing frequent sensor engagement). Robot navigation would be constrained within the inner rectangle for predictable, smooth motion.

**{ insert image of idea}**

---

## Conclusions


---

## Links

**ROS Documentation**:
- ROS Kinetic: http://wiki.ros.org/kinetic
- AMCL: http://wiki.ros.org/amcl  
- move_base: http://wiki.ros.org/move_base

**Hardware Libraries**:
- libmcp2221: https://github.com/ZakKemble/libmcp2221.git

**Apple Developer**:
- Compare Memberships: https://developer.apple.com/support/compare-memberships/

**Github repo**:https://github.com/Ribena-dev/ExpPlatform_2.git 


---


