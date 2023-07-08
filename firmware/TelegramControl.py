#!/usr/bin/env python3.6

import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt8
import string
import socket
import telepot
import telebot
import urllib.request
import urllib.error
import emoji
import sys
from time import sleep
from os import system, chdir, listdir
from telebot import types
from telepot.namedtuple import InlineKeyboardMarkup, InlineKeyboardButton
 
#Administrators chat_id
Alessandro = 486322403
Mauro = 797772394
Alessia = 894712918
Stefano = 0
Francesco = 0
Administrators = {Alessandro, Mauro, Alessia, Stefano, Francesco}

#Inizialization global variables
update_id = 1
connection = 1
shtdwn = 0
ora = None
t_value = False
manual_mode = 0
execDone = 0
battery_level = 0
light_level = 0
distance_obstacle = 1000
walking_legs = 3
humidity = 0
state = 0

#Initialization of custom keyboards
custom = types.ReplyKeyboardMarkup()
itembtn11 = types.KeyboardButton(emoji.emojize(':party_popper:', language='en'))
itembtn12 = types.KeyboardButton(emoji.emojize(':up_arrow:', language='en'))
itembtn13 = types.KeyboardButton(emoji.emojize(':counterclockwise_arrows_button:', language='en'))
itembtn21 = types.KeyboardButton(emoji.emojize(':left_arrow:', language='en'))
itembtn22 = types.KeyboardButton(emoji.emojize(':potted_plant:', language='en'))
itembtn23 = types.KeyboardButton(emoji.emojize(':right_arrow:', language='en'))
itembtn31 = types.KeyboardButton(emoji.emojize(':dagger:', language='en'))
itembtn32 = types.KeyboardButton(emoji.emojize(':down_arrow:', language='en'))
itembtn33= types.KeyboardButton(emoji.emojize(':stop_sign:', language='en'))
custom.row(itembtn11, itembtn12, itembtn13)
custom.row(itembtn21, itembtn22, itembtn23)
custom.row(itembtn31, itembtn32, itembtn33)

yn_keyboard = InlineKeyboardMarkup(inline_keyboard = [[InlineKeyboardButton(text = 'Yes', callback_data = 'Y'),InlineKeyboardButton(text = 'No', callback_data = 'N')]])
			
default = types.ReplyKeyboardRemove(selective=False)

#Node definition
rospy.init_node('telegram_input', anonymous=True) 

def callback(data):
	global execDone, state, battery_level, light_level, distance_obstacle, walking_legs, humidity
	rospy.loginfo("Read: " + data.data)
	if data.data.__eq__(str('alive')):
		bot.sendMessage(Alessandro, "I'm awake and ready to keep safe my little plant")
		execDone = 1
	elif data.data.__eq__(str('done')):
		execDone = 1
	elif state == 1 and data.data[0] == '#':
		inputs = data.data.split(' ')
		battery_level = float(inputs[1])
		light_level = float(inputs[2])
		distance_obstacle = float(inputs[3])
		walking_legs = int(inputs[4])
		humidity = float(inputs[5])
		state = 0
	elif data.data[0] == '!':
		bot.sendMessage(Alessandro, data.data[2:])
	

#Publishing definition
manualpub = rospy.Publisher("manual", UInt8, queue_size=1)
pub = rospy.Publisher("talker", UInt8, queue_size=10)
rospy.Subscriber("arduino", String, callback)

def get_ip_address():
	ip_address = '';
	s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	s.connect(("8.8.8.8",80))
	ip_address = s.getsockname()[0]
	s.close()
	return ip_address

def talk(to_print, chat_id_sender):
	global execDone
	time = rospy.get_time()
	to_publish = "\n\tFrom: " + str(chat_id_sender) + "\n\tText: '" + str(emoji.demojize(to_print)) + "' written at %s" % time
	rospy.loginfo(to_publish)
	pub.publish(to_publish)
	#pub.publish(int(to_print))
	#rospy.loginfo(to_print)
	execDone = 0
	while execDone == 0:
		sleep(0.1)

def status(chatid_sender):
	global battery_level, light_level, distance_obstacle, walking_legs, state, humidity
	manualpub.publish(int("12"))
	state = 1
	while state == 1:
		sleep(0.1)
	to_send = "BULBABOT: ONLINE\n\n"
	to_send = to_send + "- Light level:                      " + str(light_level) + "\n"
	to_send = to_send + "- Humidity:                        " + str(humidity) + "\n"
	to_send = to_send + "- Distance of obstacles:  "
	if distance_obstacle > 50:
		to_send = to_send + "No obstacles\n"
	else:
		to_send = to_send + str(distance_obstacle) + "\n"
	to_send = to_send + "- Battery level:                  " + str(battery_level) + "\n"
	to_send = to_send + "- Walking legs:                  " + str(walking_legs) + "\n"
	bot.sendMessage(chatid_sender, to_send)
    
def telegram_read(text_read, chat_id_sender):
	global manual_mode, shtdwn, execDone
	#Checking if text message is an existing command
	text_read = emoji.demojize(text_read)
	#talk(text_read, chat_id_sender)
	if text_read.__eq__(str(':potted_plant:')):
		tb.send_message(chat_id_sender, 'Autonomous Mode Enable', reply_markup=default)
		manualpub.publish(int("9"))
	elif text_read.__eq__(str(':up_arrow:')):
		bot.sendMessage(chat_id_sender, 'Front walk executing...')
		manualpub.publish(int("0"))
		execDone = 0
		while execDone == 0: #waiting for arduino response
			sleep(0.1)
		bot.sendMessage(chat_id_sender, "Done!")
	elif text_read.__eq__(str(':counterclockwise_arrows_button:')):
		bot.sendMessage(chat_id_sender, 'Rotation command')
		manualpub.publish(int("5"))
		execDone = 0
		while execDone == 0: #waiting for arduino response
			sleep(0.1)
		bot.sendMessage(chat_id_sender, 'Done!')
	elif text_read.__eq__(str(':left_arrow:')):
		bot.sendMessage(chat_id_sender, 'Left walk executing...')
		manualpub.publish(int("1"))
		execDone = 0
		while execDone == 0: #waiting for arduino response
			sleep(0.1)
		bot.sendMessage(chat_id_sender, 'Done!')
	elif text_read.__eq__(str(':party_popper:')):
		bot.sendMessage(chat_id_sender, 'Dance command')
	elif text_read.__eq__(str(':right_arrow:')):
		bot.sendMessage(chat_id_sender, 'Right walk executing...')
		manualpub.publish(int("2"))
		execDone = 0
		while execDone == 0: #waiting for arduino response
			sleep(0.1)
		bot.sendMessage(chat_id_sender, 'Done!')
	elif text_read.__eq__(str(':dagger:')):
		tosend = "A robot may not injure a human being or, through inaction, allow a human being to come to harm. Isaac Asimov"
		tosend = tosend + "\nHow could I disrespect the first robotic rule?"
		bot.sendMessage(chat_id_sender, tosend)
	elif text_read.__eq__(str(':down_arrow:')):
		bot.sendMessage(chat_id_sender, 'Back walk executing...')
		manualpub.publish(int("3"))
		execDone = 0
		while execDone == 0: #waiting for arduino response
			sleep(0.1)
		bot.sendMessage(chat_id_sender, "Done!")
	elif text_read.__eq__(str(':stop_sign:')):
		bot.sendMessage(chat_id_sender, 'Interrupting walk...')
		manualpub.publish(int("4"))
		execDone = 0
		while execDone == 0: #waiting for arduino response
			sleep(0.1)
		bot.sendMessage(chat_id_sender, "Done!")
	elif text_read.__eq__(str('/status')):
		status(chat_id_sender) 
	elif text_read.__eq__(str('/help')):
		tosend = "Manual Mode - Command List:\n\n"
		tosend = str(tosend) + emoji.emojize(':party_popper: - Makes the robot dance', language='en') + "\n"
		tosend = str(tosend) + emoji.emojize(':up_arrow: - The robot walks straight forward', language='en') + "\n"
		tosend = str(tosend) + emoji.emojize(':counterclockwise_arrows_button: - The robot turn of 180 degrees', language='en') + "\n"
		tosend = str(tosend) + emoji.emojize(':left_arrow: - The robot turn left', language='en') + "\n"
		tosend = str(tosend) + emoji.emojize(':potted_plant: - Get back to autonomous mode', language='en') + "\n"
		tosend = str(tosend) + emoji.emojize(':right_arrow: - The robot turn right', language='en') + "\n"
		tosend = str(tosend) + emoji.emojize(':dagger: - The robot attack', language='en') + "\n"
		tosend = str(tosend) + emoji.emojize(':down_arrow: - The robot walks backwards', language='en') + "\n"
		tosend = str(tosend) + emoji.emojize(':stop_sign: - The robot stop and sit', language='en') + "\n"
		tosend = str(tosend) + "\nTry also /creators to know more about me and who built me!"
		bot.sendMessage(chat_id_sender, tosend)
	elif text_read.__eq__(str('/manual')):
		tb.send_message(chat_id_sender, "Manual Mode Enabled", reply_markup=custom)
		manual_mode = 1
		manualpub.publish(int("8"))
		print("# Manual Mode Enabled")
	elif text_read.__eq__(str('/shutdown')):
		bot.sendMessage(chat_id_sender, 'Are you sure?',reply_markup=yn_keyboard)
		shtdwn = 1
	elif text_read.__eq__(str('/ip')):
		try:
			#Checking Internet Connection
			urllib.request.urlopen('https://www.google.com/', timeout = 15)
			tosend = 'I am online at: ' + get_ip_address()
			bot.sendMessage(chat_id_sender, tosend)
		except urllib.error.URLError as Error:
			#No connection 
			bot.sendMessage(chat_id_sender, 'I am not connected to internet')
	elif text_read.__eq__(str('/creators')):
		to_send = "During a unexpected hot day of May, five students of Politecnico "
		to_send = to_send + "of Turin where studying for the exam of Robotics, "
		to_send = to_send + "while a strange sound coming from a dark corner of the room "
		to_send = to_send + "caught their attention. A dehydrated plant was asking for water!"
		to_send = to_send + ":worried_face:\n"
		bot.sendMessage(chat_id_sender, emoji.emojize(to_send, language='en'))
		sleep(10)

		to_send = "After a full invogorating glass of it, watching the plant "
		to_send = to_send + "getting better, an idea lights :light_bulb:: Why "
		to_send = to_send + "not guarantee the attention that this plant need?"
		bot.sendMessage(chat_id_sender, emoji.emojize(to_send, language='en'))
		sleep(7)

		to_send = "That day, corrensponds to my first birthday :birthday_cake:. Only after few months later "
		to_send = to_send + "I was already able to walk and to provide at my protege everything"
		to_send = to_send + " it needs."
		bot.sendMessage(chat_id_sender, emoji.emojize(to_send, language='en'))
		sleep(8)

		to_send = "A big thanks to that five students :seedling:, they were my creators:\n"
		to_send = to_send + "- Alessandro Tiozzo\n- Alessia De Marco\n- Stefano Bassino\n"
		to_send = to_send + "- Mauro La Rocca\n- Francesco Gervino"
		bot.sendMessage(chat_id_sender, emoji.emojize(to_send, language='en'))
	elif text_read.__eq__(str('/debug')):
		manualpub.publish(int("11"))
		sys.exit("# Exit requested for degub issues")
	else:
		bot.sendMessage(chat_id_sender, 'Error - Unknown request')

def incoming_messages():
	global update_id, shtdwn
	#Checking if there're messages
	msg = bot.getUpdates(offset = update_id)
	if msg != []:
		update_id = msg[0]['update_id']
		update_id += 1
		try: 
			#Checking message type
			flavor = telepot.flavor(msg[0]['message'])
			if telepot.flavor(msg[0]['message']) == 'chat':
				content_type, chat_type, chat_id = telepot.glance(msg[0]['message'])
				#print(chat_id)
				nome = msg[0]["message"]["from"]["first_name"]
				toprint = str(nome) + str(" : ") + str(chat_id)
				print(toprint)
				if chat_id in Administrators:
					if content_type == 'text':
						text = msg[0]['message']['text']
						telegram_read(text, chat_id) #Evaluate the text that has been sent from telegram
					else:
				        	bot.sendMessage(chat_id, 'Error - Unknown content')
						                
		except KeyError as e:
			try:
				#Shutdown process through the callback_query confirm method
				flavor = telepot.flavor(msg[0]['callback_query'])
				from_id = msg[0]['callback_query']['message']['chat']['id']
				query_data = msg[0]['callback_query']['data']
				if from_id in Administrators:
					if shtdwn == 1:
						shtdwn = 0
						if query_data == 'Y':
							print("# Shutting down")
							bot.sendMessage(from_id, 'I am going to sleep')
							system('sudo shutdown now')
						elif query_data == 'N':
		        	    			bot.sendMessage(from_id, 'Nap cancelled')		        	    			
			except KeyError as e:
				('KeyError - Unknown Flavor')

#BulbaBot Token
TOKEN = '5359903695:AAEVr-HqTxjIMSFIChgmwq-9RiWuMuiwFtQ'
rate = rospy.Rate(1)	

if __name__ == '__main__':
	bot = telepot.Bot(TOKEN)
	tb = telebot.TeleBot(TOKEN) #used only for custom keyboard
	print("-------------------- BULBABOT IS ONLINE --------------------")
	updates = bot.getUpdates()
	if updates:
		update_id = updates[-1]['update_id']
		update_id += 1
	markup = types.ReplyKeyboardRemove(selective=False)
	tb.send_message(Alessandro, emoji.emojize("The human being is killing green life :potted_plant:\n\nMy awakening will give another day of hope :sun:", language='en'), reply_markup=markup) 	
	execDone = 0
	print("# Connecting with Arduino")
	while execDone == 0:
		manualpub.publish(int("13"))	
		sleep(1)
	print("# Connected successfully")
	print("# Autonomous Mode Enabled")
	while True:
		rate.sleep()
		try:
			incoming_messages()
		except rospy.ROSInterruptException:
			pass
