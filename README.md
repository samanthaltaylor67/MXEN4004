**Introduction**

The scope of this thesis was to evaluate the usability of a social robot integrated with generative AI conversational abilities to support children with Autism Spectral Disorder.
This thesis was completed between July 2023 to June 2024 by Samantha Taylor from Curtin University.
If you have any questions please do not hesitate to contact me at samanthaltaylor67@gmail.com.

This repo contains all code to achieve conversational functionality with MiRo using ChatGPT-4. The conversation begins when the wake statement "Hello GPT" is spoken and will end when once the ending statement "Goodbye GPT" is registered. You must populate your OpenAI API key into ```speech_recogniser/src/.env```. To initialise the system context for the GPT model, a file named "personality.txt" should be created in ```/Home/.ros```.

**The following nodes are used:**

•	*recogniser* – Uses voice activation detection (VAD) to record audio from MiRo’s right ear microphone as the user is speaking and transcribes this speech using OpenAi's Whisper into text.

•	*response* - Generates a response, with accompanying user sentiment, to the transcribed speech using OpenAI's GPT-4.

•	*streamer* – Converts the response text to audible speech and stream this output through MiRo’s speakers.

•	*lift_head* – Lift MiRo’s head as the conversation begins and lower it when the conversation ends.

•	*sentiment_analysis* – Exhibit the sentiment of the users input using the array of lights on either side of MiRo’s body.

•	*point_to_sound* – Pivot to reposition MiRo’s body to point towards the speaker*. This is only node that is not run through the roslaunch command, use ```rosrun miro_active_hearing point_to_sound.py``` in a new terminal.


**Usage**

Using Ubuntu 20.04 and python 3.8.10,

Use the following guide to install the MDK: http://labs.consequentialrobotics.com/miro-e/docs/index.php?page=Developer_Install_Steps_Install_MDK

The MDK was extracted to the Desktop, and the miro_active_hearing and speech_recogniser were installed into mdk-XXXXXX/catkin_ws
```
cd ~/mdk/catkin_ws/src
git clone https://github.com/samanthaltaylor67/MXEN4004.git
catkin build
source devel/setup.bash
```



*All credit for this node is owed to Yijing Chen, Akhil Makeswaran, Glen Palm and Alex Lucas - https://github.com/MiRo-projects/miro_active_hearing
