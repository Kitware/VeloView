#!/usr/bin/env python3

'''
Simple tool to extract pcaps from ROS bag files.
'''

import argparse
import contextlib
import logging
import os

import dpkt
import rosbag


# -----------------------------------------------------------------------------
# Globals
# -----------------------------------------------------------------------------

logger = logging.getLogger(__name__)


# -----------------------------------------------------------------------------
# Packet functions.
# -----------------------------------------------------------------------------

def wrap_ethernet_ip_udp(data):
    '''
    Wrap the data in a UDP datagram.

    TODO:
        Expose the ports, IPs and MACs as parameters if necessary.
    '''

    # UDP datagram.
    udp = dpkt.udp.UDP(
        sport=2368,  # source port
        dport=2368   # destination port
    )
    udp.data = data
    udp.ulen += len(data)

    # IP packet.
    ip = dpkt.ip.IP(
        src=bytes((192, 168, 0, 1)),      # source IP
        dst=bytes((255, 255, 255, 255)),  # destination IP
        ttl=0xff,                         # time-to-live
        p=dpkt.ip.IP_PROTO_UDP            # UDP protocol
    )
    ip.data = udp

    # Ethernet frame.
    eth = dpkt.ethernet.Ethernet(
        dst=b'\xff\xff\xff\xff\xff\xff',  # destination MAC
        src=b'\x60\x76\x88\x00\x00\x00'   # source MAC (60:76:88:00:00:00 is Velodyne)
    )
    eth.data = ip

    return eth.pack()


# -----------------------------------------------------------------------------
# Argument parsing.
# -----------------------------------------------------------------------------

def parse_args(args=None):
    '''
    Parse command-line arguments.

    Return:
        The argparse.ArgumentParser-parsed arguments.
    '''
    parser = argparse.ArgumentParser(description='Extract data from ROS bag files.')
    parser.add_argument('paths', nargs='*', metavar='<path>', help='Paths to ROS bag files.')
    parser.add_argument('-l', '--list', action='store_true', help='List the topics in each bag.')
    return parser.parse_args(args=args)


# -----------------------------------------------------------------------------
# Context management.
# -----------------------------------------------------------------------------

def context_managed_bags(paths):
    '''
    Open the bags using the rosbag. Bag context to ensure that they are properly closed when finished.

    Args:
        paths: The paths to the bag files.

    Returns:
        A generator over the bag objects.
    '''
    for path in paths:
        with rosbag.Bag(path) as bag:
            yield bag


# -----------------------------------------------------------------------------
# Commands
# -----------------------------------------------------------------------------

def list_topics(bags):
    '''
    List the topics in each bag.

    Args:
        bags: An iterable over bag objects.
    '''
    for bag in bags:
        print(bag.filename)
        topics = set(topic for (topic, _, _) in bag.read_messages())
        print('  {}'.format('\n  '.join(sorted(topics))))


def extract_packets(bags):
    '''
    Extract the packets in each bag to pcap files.

    Args:
        bags: An iterable over bag objects.
    '''

    for bag in bags:
        dirpath = '{}-pcaps'.format(bag.filename)
        os.makedirs(dirpath, exist_ok=True)
        with contextlib.ExitStack() as stack:
            writers = dict()
            for topic, message, _ in bag.read_messages():
                try:
                    packets = message.packets
                except AttributeError:
                    continue

                name = '{}.pcap'.format(topic)
                outpath = os.path.join(dirpath, name)
                try:
                    writer = writers[outpath]
                except KeyError:
                    handle = stack.enter_context(open(outpath, 'wb'))
                    writer = dpkt.pcap.Writer(handle)
                    writers[outpath] = writer
                logger.info('extracting packets from topic {} in {} to {}'.format(topic, bag.filename, outpath))
                for packet in packets:
                    stamp = packet.stamp
                    secs = float(stamp.secs) + float(stamp.nsecs) / 1e9
                    writer.writepkt(wrap_ethernet_ip_udp(packet.data), ts=secs)


# -----------------------------------------------------------------------------
# Main.
# -----------------------------------------------------------------------------

def main(args=None):
    '''
    Run the main tool.

    Args:
        args: The command-line arguments to use. If None, the arguments in sys.argv[1:] are used.
    '''
    pargs = parse_args(args=args)
    bags = context_managed_bags(pargs.paths)
    if pargs.list:
        list_topics(bags)
    else:
        extract_packets(bags)


if __name__ == '__main__':
    logging.basicConfig(
        level=logging.INFO,
        style='{',
        format='[{asctime:s}] {levelname:s}: {message:s}',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    try:
        main()
    except KeyboardInterrupt:
        pass
