-- phpMyAdmin SQL Dump
-- version 5.2.1deb1ubuntu1
-- https://www.phpmyadmin.net/
--
-- Host: localhost:3306
-- Generation Time: Oct 23, 2024 at 01:04 PM
-- Server version: 8.0.37-0ubuntu0.23.10.2
-- PHP Version: 8.2.10-2ubuntu2.2

SET SQL_MODE = "NO_AUTO_VALUE_ON_ZERO";
START TRANSACTION;
SET time_zone = "+00:00";


/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!40101 SET NAMES utf8mb4 */;

--
-- Database: `PepperProject`
--

-- --------------------------------------------------------

--
-- Table structure for table `ros_commands`
--

CREATE TABLE `ros_commands` (
  `id` int NOT NULL,
  `command_name` varchar(255) NOT NULL,
  `description` text,
  `parameters` text
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;

--
-- Dumping data for table `ros_commands`
--

INSERT INTO `ros_commands` (`id`, `command_name`, `description`, `parameters`) VALUES
(1, 'Lancer DCM', 'Lance le DCM pour connecter Pepper via ROS avec IP.', 'roslaunch pepper_dcm_bringup pepper_bringup.launch robot_ip:=192.168.1.34 roscore_ip:=localhost network_interface:=enp0s3'),
(2, 'Driver Naoqi', 'Démarre le driver Naoqi pour interagir avec Pepper.', 'roslaunch naoqi_driver naoqi_driver.launch nao_ip:=192.168.0.126 roscore_ip:=localhost network_interface:=enp0s8'),
(3, 'Calibrer caméra', 'Lance l’outil de calibration de la caméra.', 'rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/cv_camera/image_raw camera:=/cv_camera'),
(4, 'Contrôle joystick', 'Permet de contrôler Pepper via une manette (joystick).', 'roslaunch teleop_twist_joy pepper.launch'),
(5, 'Avancer', 'Fait avancer Pepper en ligne droite.', 'rostopic pub /cmd_vel geometry_msgs/Twist \"[0.5, 0, 0]\" \"[0, 0, 0]\"'),
(6, 'Reculer', 'Fait reculer Pepper en ligne droite.', 'rostopic pub /cmd_vel geometry_msgs/Twist \"[-0.5, 0, 0]\" \"[0, 0, 0]\"'),
(7, 'Tourner à gauche', 'Fait tourner Pepper vers la gauche sur un angle donné.', 'rostopic pub /cmd_vel geometry_msgs/Twist \"[0, 0, 0]\" \"[0, 0, 0.5]\"'),
(8, 'Tourner à droite', 'Fait tourner Pepper vers la droite sur un angle donné.', 'rostopic pub /cmd_vel geometry_msgs/Twist \"[0, 0, 0]\" \"[0, 0, -0.5]\"'),
(9, 'Avancer vers la gauche', 'Fait avancer Pepper tout en se déplaçant vers la gauche.', 'rostopic pub /cmd_vel geometry_msgs/Twist \"[0.5, 0.5, 0]\" \"[0, 0, 0]\"'),
(10, 'Avancer vers la droite', 'Fait avancer Pepper tout en se déplaçant vers la droite.', 'rostopic pub /cmd_vel geometry_msgs/Twist \"[0.5, -0.5, 0]\" \"[0, 0, 0]\"'),
(11, 'Tourner sur soi-même vers la gauche', 'Fait tourner Pepper sur lui-même dans le sens anti-horaire (vers la gauche).', 'rostopic pub /cmd_vel geometry_msgs/Twist \"[0, 0, 0]\" \"[0, 0, 0.5]\"'),
(12, 'Tourner sur soi-même vers la droite', 'Fait tourner Pepper sur lui-même dans le sens horaire (vers la droite).', 'rostopic pub /cmd_vel geometry_msgs/Twist \"[0, 0, 0]\" \"[0, 0, -0.5]\"'),
(13, 'Tour complet vers la gauche', 'Fait tourner Pepper sur lui-même à 360 degrés dans le sens anti-horaire.', 'rostopic pub /cmd_vel geometry_msgs/Twist \"[0, 0, 0]\" \"[0, 0, 3.14]\"'),
(14, 'Tour complet vers la droite', 'Fait tourner Pepper sur lui-même à 360 degrés dans le sens horaire.', 'rostopic pub /cmd_vel geometry_msgs/Twist \"[0, 0, 0]\" \"[0, 0, -3.14]\"'),
(15, 'Indiquer la position de la paume de la main', 'Retourne la position de la paume de la main en utilisant les joints associés.', 'rostopic echo /joint_states | grep LHand'),
(16, 'Parler', 'Fait dire un message prédéfini à Pepper.', 'rostopic pub /speech std_msgs/String \"data: \'Hello ISTY students\'\"'),
(17, 'Saluer', 'Fait lever la main de Pepper pour dire bonjour.', 'rostopic pub /joint_angles naoqi_bridge_msgs/JointAnglesWithSpeed \n     \"joint_names: [\'LShoulderRoll\', \'LShoulderPitch\'], joint_angles: [0, 1.57], speed: 0.1\"');

--
-- Indexes for dumped tables
--

--
-- Indexes for table `ros_commands`
--
ALTER TABLE `ros_commands`
  ADD PRIMARY KEY (`id`);

--
-- AUTO_INCREMENT for dumped tables
--

--
-- AUTO_INCREMENT for table `ros_commands`
--
ALTER TABLE `ros_commands`
  MODIFY `id` int NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=18;
COMMIT;

/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
