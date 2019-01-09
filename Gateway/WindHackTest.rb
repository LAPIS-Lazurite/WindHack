#! /usr/bin/ruby
# -*- coding: utf-8; mode: ruby -*-
# Function:
#   Lazurite Sub-GHz/Lazurite Pi Gateway Sample program
#   WindHackTest.rb
require 'LazGem'

laz = LazGem::Device.new

# Halt process when CTRL+C is pushed.
finish_flag=0
Signal.trap(:INT){
	finish_flag=1
}

# open device driver
laz.init()

dst_short_addr = 0x444B
ch = 50
panid = 0xabcd
baud = 100
pwr = 20
finish_fag = 0
quit_flag = false

# main routine
laz.begin(ch,panid,baud,pwr)
laz.rxEnable()

while quit_flag == false do
	p "enter: 1=start, 2=stop, 3=list file, 4=dump file 17010100.CSV, q=quit"
	input = STDIN.gets.chomp!
	if input == "1" then
		begin
			laz.send(panid,dst_short_addr,"st")
		rescue Exception => e
			p e
		end
		next
	elsif input == "2" then
		begin
			laz.send(panid,dst_short_addr,"sp")
		rescue Exception => e
			p e
		end
		next
	elsif input == "3" then
		begin
			laz.send(panid,dst_short_addr,"lf")
		rescue Exception => e
			p e
		end
	elsif input == "4" then
		begin
			laz.send(panid,dst_short_addr,"df 17010100.CSV")
		rescue Exception => e
			p e
		end
	elsif input == "q" then
		quit_flag = true 
		next
	end

	while finish_flag == 0 do
		if laz.available() <= 0
			sleep 0.01
			next
		end
		rcv = laz.read()
		# printing data
		p rcv["payload"]
		#	binary mode
		#	prtdata = rcv['payload'].unpack("H*").map {|item|item.upcase}
		#	p prtdata
	end
	finish_flag = 0
end

# finishing process
laz.close()
laz.remove()


