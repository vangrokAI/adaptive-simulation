#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import math
import random
import abc
import importlib

class FTDataProvider(abc.ABC):
    """Abstrakte Basisklasse für F/T-Sensordaten-Provider"""
    
    @abc.abstractmethod
    def get_wrench_data(self, timestamp):
        """Liefert die aktuellen F/T-Sensordaten als WrenchStamped-Objekt"""
        pass
    
    def cleanup(self):
        """Aufräumarbeiten beim Beenden"""
        pass

class ADSDataProvider(FTDataProvider):
    """Provider für TwinCAT ADS-basierte F/T-Sensordaten"""
    
    def __init__(self, node):
        self.node = node
        self.ads_available = False
        
        # Import pyads nur wenn dieser Provider verwendet wird
        try:
            import pyads
            self.pyads = pyads
            self.ads_available = True
            
            # ADS-Verbindung herstellen
            twincat_netid = node.get_parameter('twincat_netid').value
            twincat_port = node.get_parameter('twincat_port').value
            
            self.node.get_logger().info(f'Verbindung zu TwinCAT: {twincat_netid}:{twincat_port}')
            
            self.plc = pyads.Connection(twincat_netid, twincat_port)
            self.plc.open()
            node.get_logger().info(f'ADS-Verbindung hergestellt: {twincat_netid}:{twincat_port}')
            
            self.ft_data_varname = node.get_parameter('ft_data_varname').value
            self.node.get_logger().info(f'F/T-Daten Variable: {self.ft_data_varname}')
            
        except ImportError:
            node.get_logger().error('pyads konnte nicht importiert werden. '
                                   'Bitte installieren: pip install pyads')
            self.ads_available = False
        except Exception as e:
            node.get_logger().error(f'ADS-Provider konnte nicht initialisiert werden: {e}')
            self.ads_available = False
    
    def get_wrench_data(self, timestamp):
        wrench = WrenchStamped()
        wrench.header.stamp = timestamp
        wrench.header.frame_id = 'ft_sensor_frame'
        
        if not self.ads_available:
            self.node.get_logger().warn('ADS ist nicht verfügbar, gebe Nullwerte zurück')
            return wrench
        
        try:
            # Daten vom PLC lesen
            ft_data = self.plc.read_list_of_reals(self.ft_data_varname, 6)
            
            # Kraft-/Drehmomentdaten zuweisen
            wrench.wrench.force.x = ft_data[0]
            wrench.wrench.force.y = ft_data[1]
            wrench.wrench.force.z = ft_data[2]
            wrench.wrench.torque.x = ft_data[3]
            wrench.wrench.torque.y = ft_data[4]
            wrench.wrench.torque.z = ft_data[5]
            
        except Exception as e:
            self.node.get_logger().error(f'Fehler beim Lesen der ADS-Daten: {e}')
        
        return wrench
    
    def cleanup(self):
        if self.ads_available and hasattr(self, 'plc') and self.plc.is_open:
            self.node.get_logger().info('Schließe ADS-Verbindung')
            self.plc.close()

class SimDataProvider(FTDataProvider):
    """Provider für simulierte F/T-Sensordaten"""
    
    def __init__(self, node):
        self.node = node
        self.count = 0
        self.sim_mode = node.get_parameter('sim_mode').value
        self.publish_rate = float(node.get_parameter('publish_rate').value)
        
        self.node.get_logger().info(f'Simulierter F/T-Sensor im Modus: {self.sim_mode}')
        
    def get_wrench_data(self, timestamp):
        wrench = WrenchStamped()
        wrench.header.stamp = timestamp
        wrench.header.frame_id = 'ft_sensor_frame'
        
        # Simulierte Daten generieren
        if self.sim_mode == 'sine':
            # Sinusförmige Kräfte simulieren
            t = self.count / self.publish_rate
            wrench.wrench.force.z = 10.0 * math.sin(2.0 * math.pi * 0.5 * t)  # 0.5 Hz Sinus
            wrench.wrench.torque.x = 2.0 * math.sin(2.0 * math.pi * 0.2 * t)  # 0.2 Hz Sinus
            wrench.wrench.force.x = 2.0 * math.sin(2.0 * math.pi * 0.1 * t)   # 0.1 Hz Sinus
        elif self.sim_mode == 'constant':
            # Konstante Werte
            wrench.wrench.force.z = -5.0  # Konstante Kraft nach unten
        elif self.sim_mode == 'random':
            # Zufällige Werte mit etwas Rauschen
            wrench.wrench.force.x = random.uniform(-1.0, 1.0)
            wrench.wrench.force.y = random.uniform(-1.0, 1.0)
            wrench.wrench.force.z = -5.0 + random.uniform(-0.5, 0.5)
            wrench.wrench.torque.x = random.uniform(-0.2, 0.2)
            wrench.wrench.torque.y = random.uniform(-0.2, 0.2)
            wrench.wrench.torque.z = random.uniform(-0.2, 0.2)
        
        self.count += 1
        return wrench

class FTSensorNode(Node):
    """ROS2-Node für F/T-Sensordaten über ADS oder Simulation"""
    
    def __init__(self):
        super().__init__('ft_sensor')
        
        # Allgemeine Parameter
        self.declare_parameter('data_source', 'sim')  # 'ads' oder 'sim'
        self.declare_parameter('publish_rate', 500.0)  # Hz
        
        # Parameter für ADS
        self.declare_parameter('twincat_netid', '192.168.2.100.1.1')
        self.declare_parameter('twincat_port', 851)
        self.declare_parameter('ft_data_varname', 'MAIN.FT_Data')
        
        # Parameter für Simulation
        self.declare_parameter('sim_mode', 'sine')  # 'sine', 'constant', 'random'
        
        # Lese Parameter
        self.data_source = self.get_parameter('data_source').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        
        # Publisher
        self.ft_pub = self.create_publisher(WrenchStamped, 'ft_sensor/wrench', 10)
        
        # Datenquelle auswählen
        if self.data_source == 'ads':
            self.get_logger().info('Verwende ADS als Datenquelle')
            self.data_provider = ADSDataProvider(self)
        else:
            self.get_logger().info('Verwende Simulation als Datenquelle')
            self.data_provider = SimDataProvider(self)
        
        # Timer für regelmäßiges Veröffentlichen
        self.timer = self.create_timer(1.0/self.publish_rate, self.timer_callback)
    
    def timer_callback(self):
        """Timer-Callback zum Veröffentlichen der F/T-Daten"""
        timestamp = self.get_clock().now().to_msg()
        wrench = self.data_provider.get_wrench_data(timestamp)
        
        # Nachricht veröffentlichen
        self.ft_pub.publish(wrench)
    
    def __del__(self):
        """Destruktor"""
        if hasattr(self, 'data_provider'):
            self.data_provider.cleanup()

def main(args=None):
    """Hauptfunktion"""
    rclpy.init(args=args)
    
    try:
        node = FTSensorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Unerwarteter Fehler: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
