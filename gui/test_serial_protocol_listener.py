#!/usr/bin/env python3
"""
Test harness for the serial_protocol listener-driven ACK flow.

Mocks a serial connection to verify:
1. wait_for_ack() returns successfully when a matching ACK is received
2. wait_for_ack() times out and raises ProtocolError when no ACK arrives
3. Unmatched ACKs are ignored and do not cause false positives

Run with: python -m pytest gui/test_serial_protocol_listener.py -v
Or directly: python gui/test_serial_protocol_listener.py
"""

import threading
import time
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import serial_protocol
from serial_protocol import ProtocolError


class MockSerialConnection:
    """Mock serial connection that simulates Teensy responses."""
    
    def __init__(self):
        self._buffer = b""
        self._buffer_lock = threading.Lock()
        self._written_data = []
        self._closed = False
    
    @property
    def in_waiting(self) -> int:
        """Return number of bytes available to read."""
        with self._buffer_lock:
            return len(self._buffer)
    
    def read(self, size: int = 1) -> bytes:
        """Read up to size bytes from the buffer."""
        with self._buffer_lock:
            data = self._buffer[:size]
            self._buffer = self._buffer[size:]
            return data
    
    def write(self, data: bytes) -> int:
        """Write data (record for verification)."""
        self._written_data.append(data)
        return len(data)
    
    def flush(self) -> None:
        """Flush output (no-op for mock)."""
        pass
    
    def close(self) -> None:
        """Close the connection."""
        self._closed = True
    
    def inject_response(self, response: str) -> None:
        """Inject a response line into the read buffer (simulates Teensy sending data)."""
        with self._buffer_lock:
            self._buffer += (response + "\n").encode('utf-8')
    
    def get_written_packets(self) -> list:
        """Return list of packets that were written."""
        return [d.decode('utf-8').strip() for d in self._written_data]


def test_wait_for_ack_success():
    """Test that wait_for_ack returns successfully when matching ACK arrives."""
    print("Test 1: wait_for_ack success with matching ACK")
    
    mock_conn = MockSerialConnection()
    
    # Start listener
    serial_protocol.start_listener(mock_conn)
    time.sleep(0.05)  # Let listener start
    
    try:
        # Build and send a packet
        sent_packet = "TYPE=CMD,CMD=SET_MODE,MODE=2\n"
        serial_protocol.send_packet(mock_conn, sent_packet)
        
        # Inject matching ACK after a brief delay (simulate Teensy response)
        def inject_ack():
            time.sleep(0.1)
            mock_conn.inject_response("TYPE=ACK,CMD=SET_MODE,MODE=2")
        
        ack_thread = threading.Thread(target=inject_ack)
        ack_thread.start()
        
        # Wait for ACK - should succeed
        result = serial_protocol.wait_for_ack(sent_packet, timeout=2.0)
        ack_thread.join()
        
        assert result is True, "wait_for_ack should return True"
        print("  PASSED: wait_for_ack returned True for matching ACK")
        
    finally:
        serial_protocol.stop_listener()


def test_wait_for_ack_timeout():
    """Test that wait_for_ack raises ProtocolError on timeout."""
    print("Test 2: wait_for_ack timeout when no ACK arrives")
    
    mock_conn = MockSerialConnection()
    
    # Start listener
    serial_protocol.start_listener(mock_conn)
    time.sleep(0.05)
    
    try:
        sent_packet = "TYPE=CMD,CMD=ESTOP,STOP=ALL\n"
        serial_protocol.send_packet(mock_conn, sent_packet)
        
        # Don't inject any ACK - should timeout
        try:
            serial_protocol.wait_for_ack(sent_packet, timeout=0.3)
            assert False, "Should have raised ProtocolError"
        except ProtocolError as e:
            assert "Timeout" in str(e), f"Expected timeout error, got: {e}"
            print("  PASSED: ProtocolError raised on timeout")
            
    finally:
        serial_protocol.stop_listener()


def test_wait_for_ack_ignores_unmatched():
    """Test that unmatched ACKs are ignored and don't cause false positives."""
    print("Test 3: wait_for_ack ignores unmatched ACKs")
    
    mock_conn = MockSerialConnection()
    
    # Start listener
    serial_protocol.start_listener(mock_conn)
    time.sleep(0.05)
    
    try:
        sent_packet = "TYPE=CMD,CMD=SET_MODE,MODE=2\n"
        serial_protocol.send_packet(mock_conn, sent_packet)
        
        # Inject wrong ACK first, then correct ACK
        def inject_acks():
            time.sleep(0.05)
            # Wrong ACK (different MODE value)
            mock_conn.inject_response("TYPE=ACK,CMD=SET_MODE,MODE=3")
            time.sleep(0.1)
            # Correct ACK
            mock_conn.inject_response("TYPE=ACK,CMD=SET_MODE,MODE=2")
        
        ack_thread = threading.Thread(target=inject_acks)
        ack_thread.start()
        
        # Wait for ACK - should eventually succeed with correct ACK
        result = serial_protocol.wait_for_ack(sent_packet, timeout=2.0)
        ack_thread.join()
        
        assert result is True, "wait_for_ack should return True after correct ACK"
        print("  PASSED: Unmatched ACK ignored, correct ACK accepted")
        
    finally:
        serial_protocol.stop_listener()


def test_send_packet_writes_correctly():
    """Test that send_packet writes the packet with proper newline."""
    print("Test 4: send_packet writes correctly")
    
    mock_conn = MockSerialConnection()
    
    # Test without trailing newline
    serial_protocol.send_packet(mock_conn, "TYPE=CMD,CMD=ESTOP,STOP=ALL")
    
    # Test with trailing newline
    serial_protocol.send_packet(mock_conn, "TYPE=CMD,CMD=SET_MODE,MODE=1\n")
    
    written = mock_conn.get_written_packets()
    assert len(written) == 2, f"Expected 2 packets, got {len(written)}"
    assert written[0] == "TYPE=CMD,CMD=ESTOP,STOP=ALL", f"First packet wrong: {written[0]}"
    assert written[1] == "TYPE=CMD,CMD=SET_MODE,MODE=1", f"Second packet wrong: {written[1]}"
    
    print("  PASSED: send_packet writes correctly with newline normalization")


def test_listener_handles_data_packets():
    """Test that DATA packets are routed to telemetry handler."""
    print("Test 5: listener routes DATA packets to telemetry handler")
    
    mock_conn = MockSerialConnection()
    received_telemetry = []
    
    def telemetry_handler(line: str):
        received_telemetry.append(line)
    
    serial_protocol.set_telemetry_handler(telemetry_handler)
    serial_protocol.start_listener(mock_conn)
    time.sleep(0.05)
    
    try:
        # Inject DATA packet
        mock_conn.inject_response("TYPE=DATA,CMD=JOINT_ANGLES,J1=45.0,J2=30.0")
        time.sleep(0.2)  # Wait for listener to process
        
        assert len(received_telemetry) >= 1, "Telemetry handler not called"
        assert "TYPE=DATA" in received_telemetry[0], f"Wrong data: {received_telemetry[0]}"
        print("  PASSED: DATA packets routed to telemetry handler")
        
    finally:
        serial_protocol.stop_listener()
        serial_protocol.set_telemetry_handler(None)


def test_listener_handles_malformed_packets():
    """Test that malformed packets don't crash the listener."""
    print("Test 6: listener handles malformed packets gracefully")
    
    mock_conn = MockSerialConnection()
    
    serial_protocol.start_listener(mock_conn)
    time.sleep(0.05)
    
    try:
        # Inject malformed packet
        mock_conn.inject_response("THIS_IS_NOT_A_VALID_PACKET")
        time.sleep(0.1)
        
        # Inject valid ACK after malformed
        mock_conn.inject_response("TYPE=ACK,CMD=TEST,VAL=1")
        time.sleep(0.1)
        
        # Verify listener is still running by checking current_ack
        with serial_protocol.current_ack_lock:
            ack = serial_protocol.current_ack
        
        assert ack is not None, "Listener stopped processing after malformed packet"
        assert "TYPE=ACK" in ack, f"Wrong ACK: {ack}"
        print("  PASSED: Listener continues after malformed packet")
        
    finally:
        serial_protocol.stop_listener()


def run_all_tests():
    """Run all tests and report results."""
    print("=" * 60)
    print("Serial Protocol Listener Tests")
    print("=" * 60)
    print()
    
    tests = [
        test_send_packet_writes_correctly,
        test_wait_for_ack_success,
        test_wait_for_ack_timeout,
        test_wait_for_ack_ignores_unmatched,
        test_listener_handles_data_packets,
        test_listener_handles_malformed_packets,
    ]
    
    passed = 0
    failed = 0
    
    for test in tests:
        try:
            test()
            passed += 1
        except Exception as e:
            print(f"  FAILED: {e}")
            failed += 1
        print()
    
    print("=" * 60)
    print(f"Results: {passed} passed, {failed} failed")
    print("=" * 60)
    
    return failed == 0


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)
