আমার self driving robot orbibot টা তে বেশ কিছু চেঞ্জ করে নতুন ডিজাইন করেছি package গুলোকে ।  
Raspberry pi 5, Yahboom Ros expansion board with IMU sensor, motor driver, 520 333rpm encoder motor, mechanical wheel, RPlidar A1, Intel Realsense D435. 

ROS2 Jazzy Base Install on Raspberry PI 5 (Ubuntu 24.04 Server). 
ROS2 Jazzy Desktop Installed on my Computer ( Ubuntu 24.04 Desktop ).
ROS2 doamin id = 42 

রোবটে যাতে প্রেশার কম পরে তাই আমি rviz সহ সব ধরনের visualization আমার Computer এ করবো, আমি আপাতত gazebo ইউজ করছি না, তাই use sim time বাদ দিছি। launch ফাইল গুলো সিম্পলে ও ক্লিন রাখতে আমার ভাল লাগে। 

প্রতিটা package এ আলাদা আলাদা launch ফাইল দিয়ে রান করবো, যাতে টেস্ট করতে সুবিধা হয়। 

description,launch.py দিয়ে robot_state launch করবো 
hardware.luanch.py দিয়ে আমি hardware launch করবো, সাথে motor enable করবো 
control.launch.py দিয়ে control system run  করবো 

একই ভাবে snesors, localization, navigation, slam সব গুলোকে আলাদা আলাদা নোড ও পারামস ফাইল ও launch ফাইলে ভাগ হতে হবে ।  
একটা একটা আমার সব গুলো package ঠিক আছে কিনা চেক করো 
আমার সাথে এখন রোবট নাই, তাই আপাতত রোবট দিয়ে চেক করা যাবে না। 