#!/usr/bin/env ruby

require 'optparse'

# The physical interface
iface = ARGV[0] 

# Default options
options = {:iface => "eth0"}

OptionParser.new do |opts|
  opts.banner = "Usage: sudo ./src_tc_stop.rb [options]\n\n" +
    "Example:\n  sudo ./src_tc_stop.rb -i eth0\n\n" +
    "Options:"

  opts.on('-i', '--iface INTERFACE', "Ethernet interface name. Default=#{options[:iface]}") { |v|
    options[:iface] = v
  }

end.parse!

# Clear tc
`tc qdisc del dev #{options[:iface]} root`
`tc qdisc del dev ifb0 root`


