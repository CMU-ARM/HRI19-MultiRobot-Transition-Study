#! /usr/bin/python

from lab_polly_speech import PollySpeech
from lab_baxter_common import traj_playback
from lab_baxter_common.general_toolkit import(
    move_to_posture
)
from lab_ros_speech_to_text.msg import(
    Speech
)
import math
import baxter_interface
import yaml
import rospy
import os 
import rospkg
from multibot_relay import (
    MultiBotInterface
)
from snips_nlu_ros import (
    SnipsNLU
)
import threading
import numpy as np
from std_srvs.srv import SetBool
from noise import pnoise1
from lab_baxter_head_tools import FaceController


class Interpretor():

    def __init__(self):

        self._speak_ctr = PollySpeech()
        self._mbot_relay = MultiBotInterface()
        self._face_ctr = FaceController()

        #enable robot
        enable_robot = baxter_interface.RobotEnable()
        enable_robot.enable()

        rospack = rospkg.RosPack()
        self._package_path = rospack.get_path('hri19_multirobot_transition_study')
        
        #to deal with SST
        rospy.Subscriber('stt', Speech, self._stt_callback, queue_size=1)
        self._stt_toggle = rospy.ServiceProxy('toggle_stt', SetBool)
        self._last_sentence = ""
        self._stt_condition = threading.Condition()
        self._stt_event = threading.Event()
        self._pause_flag = True
        self._nlu_engine = SnipsNLU()
    
        self._head = baxter_interface.Head()

        self._running = True

        #memory for each session
        self._temp_memory = dict()
        self._memory_stack = list()

        self._index_stack = list()
        self._index = 0 

        """ The following are the list of actions/behaviors
        that exist for the robot, each action's name directly correspond
        to the same name of the function
        """
        self._operation_list = ['speak', 'move_to_posture', 'move_head', 'change_face','natural_language_understanding','wait_response','wait','yes_no_question', 
            'signal_robot', 'wait_for_signal','run_file','choice_question','load_param']
        rospy.loginfo("RUN Complete Initiailization")


    def _stt_callback(self, msg):
        if not self._pause_flag and msg.is_final:
            with self._stt_condition:
                self._last_sentence = msg.text
                self._stt_condition.notify_all()

    def _wait_for_valid_text_phrases(self, eval_function, timeout=rospy.Duration(secs=10)):

        self._last_sentence = ""
        #start listening to audio
        self._stt_toggle(True)  
        rospy.logdebug('start listening for audio')
        self._pause_flag = False
        done = False
        start_time = rospy.Time.now()
        while not self._pause_flag:
            self._stt_condition.acquire()
            self._stt_condition.wait(0.5)
            if self._last_sentence == "":
                #timed out
                if not rospy.is_shutdown() and timeout.to_sec() != 0 and (rospy.Time.now() - start_time) > timeout:
                    #timeout time reached, quiting ....
                    self._pause_flag = True
                    self._stt_condition.release()
                    self._stt_toggle(False)  
                    return None
                #there is still time left on the general loop, repeat
                #the whole loop
                self._stt_condition.release()
            else:
                #There's a valid sentence
                arg = self._last_sentence
                self._last_sentence = ""
                self._stt_condition.release()
                #evaluate the string
                print(self._last_sentence)
                success, result = eval_function(arg)
                print(success)
                print(result)
                if success:
                    self._pause_flag = False
                    self._stt_toggle(False)  
                    return result
                #the evaluation function was inconclusive
                #continue to loop

    def resolve_file_path(self, filename):
        """Check the file path of the filename
        (1) If no directory given, we automatically search
        /hri19_multirobot_transition_study/res/ for it

        returns
        -------
        complete_file_path : str
            Complete file path, None if the file doesn't exist

        """
        dir_path, file_last_name = os.path.split(filename)
        if dir_path == '':
            file_complete_path = os.path.join(self._package_path,'res', filename)
            return file_complete_path if os.path.exists(file_complete_path) else None
        else:
            raise NotImplementedError('Other forms of path resolution is not implemented')

    def _function_exec_check(self, op, param_obj):
        """
        """
        try:
            rtn_flag = getattr(self, op)(**param_obj)
            return (rtn_flag if rtn_flag is not None else True)
        except TypeError as terr:
            print(terr)
            print("Incorrect Argunments for {}(): Given Arguments are:{}".format(op, param_obj.keys()))


    def parse(self, _filename, restart_mem=False):
        """ Play the file from the given file name
        """

        #restart memory
        if restart_mem:
            self._temp_memory = dict()

        #first resolve the filename according to the rules 
        filename = self.resolve_file_path(_filename)
        if filename == None:
            rospy.logwarn('Unable to find file named {}'.format(_filename))
            return False

        #open the file and get a list of actions that we cared off.
        action_list = []
        with open(filename, 'r') as fs:
            instruction_set = yaml.load(fs)
            action_list = instruction_set['actions']
            #if there is memory load if it exist
            if 'memory' in instruction_set:
                self._temp_memory.update(instruction_set['memory'])

        self._index = 0
        while True:
            if self._index < 0 or self._index >= len(action_list):
                break
            action = action_list[self._index]
            if action['op'] == 'end':
                break
            rospy.loginfo('RUN {} in {} || op:{} param:{}'.format(self._index, _filename, action['op'],action['param']))
            #(1) Make sure the action is the list of available action
            if action['op'] in self._operation_list:
                #update the next index
                self._index += 1
                #call the associated function
                return_flag = self._function_exec_check(action['op'], action['param'] if 'param' in action else dict())
                if not return_flag:
                    #This means that the action has failed
                    if 'fail' in action:
                        fail_options = action['fail']
                        if 'run_file' in fail_options:
                            self._function_exec_check('run_file', fail_options['run_file']['param'] if 'param' in fail_options['run_file'] else dict())
                        if 'goto' in fail_options:
                            if action['fail']['goto'] != 'pass':
                                self._index = action['fail']['goto']
                
            else:
                #action isn't available
                rospy.logwarn('Unavailable action {} called in parse function'.format(action['op']))
                self._index += 1


    """ Anything below are actions that are allowed """

    def speak(self, text, **kwargs):

        #add formatting if it exist
        if 'format' in kwargs:
            format_list = []
            for format in kwargs['format']:
                if type(format) is dict:
                    format_list.append(self._temp_memory[format['input_key']])
                elif type(format) is str:
                    format_list.append(format)
            text = text.format(*format_list)
        #speak the sentence
        self._speak_ctr.speak(text, **kwargs)

    def wait(self, time, **kwargs):
        #sleep for time
        rospy.sleep(rospy.Duration.from_sec(time))

    def natural_language_understanding(self, NLU_type, output_keys, **kwargs):
        """ Which NLU parser to use, we then link the outputs to the associated output keys
        """
        #(1) Run the NLU engine
        #(2) wait for the key
        #(3) save it to the output keys

        init_threshold = 0 #0.25 #no threshold
        min_threshold = 0 #0.25
        decay = 0.05 #linear decay if need to
        if 'threshold' in kwargs:
            init_threshold = kwargs['threshold']
        threshold = init_threshold


        timeout_duration = rospy.Duration() 
        if 'timeout' in kwargs:
            timeout_duration = rospy.Duration(kwargs['timeout'])


        start_time = rospy.Time.now()

        def eval_function(sentence):
            print(sentence)
            name, prob, slot_info = self._nlu_engine.parse(sentence)
            print(name)
            print(prob)
            print(slot_info)

            diff_time = (rospy.Time.now() - start_time).to_sec()
            threshold = np.max([0, (init_threshold - diff_time*decay)])

            if name == NLU_type and prob >= threshold:
                if slot_info != None and len(slot_info) > 0:
                    return True, slot_info
                else:
                    return False, None
            else:
                return False, None            

        slot_info  = self._wait_for_valid_text_phrases(eval_function, timeout=timeout_duration)

        if slot_info is not None:
            for i in range(0, len(output_keys)):
                self._temp_memory[output_keys[i]] = slot_info[i]['rawValue']

        return True if slot_info is not None else False

    def wait_response(self, timeout):
        """Wait for the user to say something
        """
        timeout_duration = rospy.Duration() if timeout == 0 else rospy.Duration(timeout)
        def eval_func(sentence):
            print(sentence)
            return True, True
        return self._wait_for_valid_text_phrases(eval_func, timeout=timeout_duration)
        

    def signal_robot(self, signal, **kwargs):
        """Signal the other robot that you have something
        """
        self._mbot_relay.send_signal(signal)

    def wait_for_signal(self, robot_id, signal, **kwargs):
        self._mbot_relay.wait_for_signal(signal, _id=robot_id)

    def yes_no_question(self, timeout, true_action, false_action):
        timeout_duration = rospy.Duration() if timeout == 0 else rospy.Duration(timeout)

        yes_words = ['yes', 'yep', 'yeah','yay','sure','yup','correct','right','okay', 'indeed','alright', 'exactly','precisely']
        no_words = ['no', 'nope', "don't",'nah','wrong','incorrect']

        def eval_func(sentence):
            print("EVAL:{}".format(sentence))
            arr_str = sentence.lower().split(' ')
            for word in arr_str:
                if word in yes_words:
                    return True, 'true'
                elif word in no_words:
                    return True, 'false'
                else:
                    return False, None

        resp = self._wait_for_valid_text_phrases(eval_func, timeout=timeout_duration)
        if resp == 'true':
            self._index = self._index if true_action == 'pass' else true_action
            return True
        elif resp == 'false':
            self._index = self._index if false_action == 'pass' else false_action
            return True
        elif resp is None:
            #This means it failed
            return False

    def choice_question(self, options, output_key, timeout):
        timeout_duration = rospy.Duration() if timeout == 0 else rospy.Duration(timeout)

        def eval_func(sentence):
            print("EVAL:{}".format(sentence))
            arr_str = sentence.lower().split(' ')
            print(arr_str)
            for word in arr_str:
                for i, option in enumerate(options):
                    if word == option['key']:
                        return True, i
                    elif word in option['alternatives']:
                        return True, i  
            return False, None

        option_idx = self._wait_for_valid_text_phrases(eval_func, timeout=timeout_duration)
        if option_idx is not None:
            self._temp_memory[output_key] = options[option_idx]['key']
            return True
        else:
            return False


    def run_file(self, filename, carry_mem, **kwargs):
        """Execute the another behavior file and whether the memory carries over
        """
        #store the old memory on a stack
        if not carry_mem:
            self._memory_stack.append(self._temp_memory)

        self._index_stack.append(self._index)

        #run the code
        success_flag = self.parse(filename, restart_mem=(not carry_mem))
        
        #push back the memory
        if not carry_mem:
            self._temp_memory = self._memory_stack.pop()

        self._index = self._index_stack.pop()

        return success_flag


    def move_to_posture(self, posture, **kwargs):
        move_to_posture(posture, record_path='{}'.format(os.path.join(self._package_path,'scripts','posture_records.yaml')))

    def move_head(self, angle ,**kwargs):
        #[print(angle)
        block = True
        speed = 1
        if 'block' in kwargs:
            block = kwargs['block']
        if 'speed' in kwargs:
            speed = kwargs['speed']

        self._head_angle = angle
        self._head_speed = speed

        # ht = threading.Thread(target=self._head.set_pan, args=(angle,speed))
        # ht.start()
        # if block:
        #     ht.join()
    
    def change_face(self, face_type):
        """ Change Baxter's face 
        """
        self._face_ctr.set_emotion(face_type)

    def load_param(self, rosparam_key, mem_key):
        
        val = rospy.get_param(rosparam_key)
        self._temp_memory[mem_key] = val

        return True


    def _head_controller(self):
        
        max_diff = 0.1
        max_noise = 0.07
        min_noise = -0.07
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and self._running:
            t = (rospy.Time.now() - start_time).to_sec()
            noise = pnoise1(t) * max_diff
            noise = (np.min([max_noise,np.max([noise,min_noise])]))
            head_angle = self._head_angle + noise   
            #print(head_angle)
            self._head.set_pan(head_angle, self._head_speed)
            rate.sleep()

    def start_head(self, init_angle=0):
        self._head_angle = init_angle
        self._head_speed = 1

        self._head_t = threading.Thread(target=self._head_controller)
        self._head_t.start()

    def close(self):
        self._running = False
        self._head_t.join()
        rospy.loginfo('Stopping Interpretor')


def main():
    rospy.init_node('run_test')

    inter = Interpretor()
    #start head controller
    inter.start_head()
    #get the file to be read
    start_parse_name = rospy.get_param('interpretor_filepath')
    rospy.loginfo("Running:{}".format(start_parse_name))
    inter.parse(start_parse_name)

    print("reached end of main")
    inter.close()


if __name__ == '__main__':
    main()