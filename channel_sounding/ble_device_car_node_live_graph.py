
""" New code that also gets ride of outlies so should be no more 4*10^9 cm anymore. Newest version"""
"""
ble_device_car_node_with_distance.py

This example demonstrates how to use the BleDevice API to perform Channel Sounding (CS) operations between
Out-Of-The-Box BLE devices, specifically between a "car node" and a "key node" examples
The script is designed to scan for peer devices, connect and pair with them, and then execute Channel Sounding
procedures to measure distance or/and collect raw data.

Main Features:
--------------
1. **Peer Discovery and Connection**:
   - Scans for BLE devices advertising specific names (see `peer_names` in `main()`).
   - Connects and pairs with the discovered peer devices.

2. **Channel Sounding (CS) Configuration**:
   - Configures CS parameters for each connection.
   - Supports both manual and automatic CS configuration modes (automatic mode still in development).

3. **CS Procedure Execution**:
   - Initiates CS procedures to collect distance measurements and/or raw channel data.
   - Supports repeated measurements.
   - Allows configuration of which events/results to wait for (distance, raw subevents, remote results, etc.).
"""

import os
import datetime
import sys
import time
import threading
import matplotlib.pyplot as plt

# Uncomment line below for local debug of packages
# sys.path.append(r"../unpi")
# sys.path.append(r"../rtls")
# sys.path.append(r"../ble_device")

from ble_device import (
    BleDevice,
    BleDeviceLoggingLevel,
    PeripheralEventType,
    ConnectionEventType,
    AddressType,
    PairingEventType,
    CentralEventType,
    CsEventType,
    CsProcedureDoneStatusType,
    CsSubeventDoneStatusType,
)

cs_config_params = {
    "conn_handle": 0,
    "config_id": 0,
    "create_context": 1,
    "main_mode": 2,
    "sub_mode": 0xFF,
    "main_mode_min_steps": 0,
    "main_mode_max_steps": 0,
    "main_mode_repetition": 0,
    "mode_zero_steps": 3,
    "role": 0,  # 0 - initiator, 1 - reflector
    "rtt_type": 0,
    "cs_sync_phy": 1,
    "channel_map": [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF],
    "channel_map_repetition": 1,
    "ch_sel": 0,
    "ch3c_shape": 0,
    "ch3C_jump": 0,
}

cs_set_procedure_params = {
    "conn_handle": 0,
    "config_id": 0,
    "max_procedure_duration": 0xFFFF,
    "min_procedure_interval": 0,
    "max_procedure_interval": 0,
    "max_procedure_count": 1,
    "min_sub_event_length": 55000,
    "max_sub_event_length": 55000,
    "aci": 0,
    "phy": 1,
    "tx_power_delta": 0x80,
    # Bitmap. set number of bits on as the peer's number of antennas (not more)
    "preferred_peer_antenna": 0b0001,
    "snr_ctrl_i": 0xFF,
    "snr_ctrl_r": 0xFF,
    "enable": 0,
}

cs_set_procedure_params_repeat = {
    "conn_handle": 0,
    "config_id": 0,
    "max_procedure_duration": 0xFFFF,
    "min_procedure_interval": 15,
    "max_procedure_interval": 15,
    "max_procedure_count": 10,
    "min_sub_event_length": 55000,
    "max_sub_event_length": 55000,
    "aci": 0,
    "phy": 1,
    "tx_power_delta": 0x80,
    # Bitmap. set number of bits on as the peer's number of antennas (not more)
    "preferred_peer_antenna": 0b0001,
    "snr_ctrl_i": 0xFF,
    "snr_ctrl_r": 0xFF,
    "enable": 0,
}


def get_logging_file_path():
    data_time = datetime.datetime.now().strftime("%m_%d_%Y_%H_%M_%S")

    logging_file_path = os.path.join(
        os.path.curdir, os.path.basename(__file__).replace(".py", "_log")
    )

    if not os.path.isdir(logging_file_path):
        os.makedirs(logging_file_path)

    logging_file = os.path.join(
        logging_file_path,
        f"{data_time}_{os.path.basename(__file__).replace('.py', '.log')}",
    )

    return os.path.abspath(logging_file)


def print_connection_data(connection_data):
    print("Connection Data")
    print("----------------------------------------------------------------------")
    print(f"Status:                 {connection_data['status']}")
    print(f"Opcode:                 {connection_data['opcode']}")
    print(
        f"Address Type:           {AddressType(connection_data['address_type']).name}"
    )
    print(
        f"Device Address:         {':'.join(format(x, '02X') for x in reversed(connection_data['dev_address']))}"
    )
    print(f"Connection Handle:      {connection_data['connection_handle']}")
    print(f"Connection Role:        {connection_data['connection_role']}")
    print(f"Connection Interval:    {connection_data['connection_interval']}")
    print(f"Connection Latency:     {connection_data['connection_latency']}")
    print(f"Connection Timeout:     {connection_data['connection_timeout']}")
    print(f"Clock Accuracy:         {connection_data['clock_accuracy']}")
    print("----------------------------------------------------------------------")


def print_pairing_data(pairing_data):
    print("Pairing Complete")
    print(f"Connection Handle:     {pairing_data['connection_handle']}")
    print(f"State:                 {pairing_data['state']}")
    print(f"Status:                {pairing_data['status']}")
    print("------------------------------------------------------")


def print_obj_data(caption, data):
    print(caption)
    print("----------------------------------------------------------------------")
    for key, value in data.items():
        print(f"{key}: {value}")
    print("----------------------------------------------------------------------")


def search_peer_name(peer_names: list, adv_data: bytes) -> str:
    i = 0
    while i < len(adv_data):
        length = adv_data[i]
        if length == 0:
            return None
        field_type = adv_data[i + 1]
        if field_type == 0x09:  # Complete Local Name
            try:
                name = bytes(adv_data[i + 2: i + 1 + length]).decode("utf-8")
            except UnicodeDecodeError:
                return None
            print(f"adv local name: {name}")
            if name in peer_names:
                return name
        i += length + 1

    return None


""" new code: function """
def search_and_connect_to_peer(car_node: BleDevice, peer_names: list, max_retries: int = 5) -> dict:
    """
    NEW IMPLEMENTATION SUMMARY:
    1. Retries scanning & connecting up to `max_retries` (5 times) instead of instantly crashing on signal drop.
    2. Extended `pairing_timeout` from 10 seconds to 30 seconds to allow slow pairing inside wooden/metal cabinets.
    """
    connections = {}

    for peer_target in peer_names:
        connected_and_paired = False

        for attempt in range(1, max_retries + 1):
            print(f"\n--- [Attempt {attempt}/{max_retries}] Scanning for peer: '{peer_target}' ---")
            
            # Start scanning
            car_node.central.start_scan(0, 0, 0)
            continue_scan = True
            connection_data = None

            while continue_scan:
                time.sleep(1e-12)

                if car_node.all_event_list.is_event_in_list(CentralEventType.NWP_ADV_REPORT):
                    adv_report = car_node.all_event_list.get_event_from_list(CentralEventType.NWP_ADV_REPORT)
                    if adv_report:
                        peer_name = search_peer_name([peer_target], adv_report["data"])
                        if peer_name is not None:
                            car_node.central.stop_scan()
                            continue_scan = False

                            peer_addr_type = adv_report["address_type"]
                            peer_address = adv_report["address"].copy()
                            print(f"Create a connection with {peer_address} ({peer_name}) as central...")
                            car_node.central.connect(peer_addr_type, peer_address, 1, 0)
                            
                            connection_data = car_node.wait_for_event(
                                event_type=ConnectionEventType.NWP_LINK_ESTABLISHED_EVENT,
                                timeout=60,
                            )

            if not connection_data:
                print(f"ERROR: Connection link establishment failed for {peer_target}. Retrying...")
                continue

            print_connection_data(connection_data)

            # Wait for pairing events with an extended 30-second timeout
            pairing_data = None
            start_time = time.time()
            pairing_timeout = 30  # Extended timeout for high-attenuation environments
            
            print(f"Waiting for pairing events from peer {peer_target} (timeout: {pairing_timeout}s)", end="", flush=True)
            while True:
                print(".", end="", flush=True)
                event_type, event_data = car_node.wait_for_any_event(timeout=0.2)

                if event_type is not None and event_data is not None:
                    if event_type == PairingEventType.NWP_PAIRING_STATE_COMPLETE:
                        print("\nPairing Complete Event Received")
                        pairing_data = event_data
                        break
                    elif event_type == PairingEventType.NWP_PAIRING_STATE_ENCRYPTED:
                        print("\nPairing Encrypted Event Received")
                        pairing_data = event_data
                        break

                if time.time() - start_time > pairing_timeout:
                    print(f"\nTimeout waiting for pairing events ({pairing_timeout} seconds)")
                    break

            if pairing_data and pairing_data.get("status") == 0:
                print_pairing_data(pairing_data)
                connections[peer_target] = connection_data["connection_handle"]
                connected_and_paired = True
                break  # Successfully connected and paired
            else:
                print(f"\nPairing failed on attempt {attempt}. Retrying connection...")
                time.sleep(1)

        if not connected_and_paired:
            print(f"ERROR: Failed to connect and pair with {peer_target} after {max_retries} attempts!")
            return None

    return connections
"""new code ends here"""


""" new code: function """
def start_cs_config(car_node, conn_handle, auto_config=False):
    """
    NEW IMPLEMENTATION SUMMARY:
    1. Boosts default BLE transmit power setting to maximum (`max_tx_power = 10` dBm).
    2. Helps the signal pierce through cabinet doors and wall obstacles.
    """
    if not auto_config:
        print("***** read local cap *****")
        car_node.cs.read_local_supported_capabilities()
        time.sleep(1)

        print("***** read remote cap *****")
        car_node.cs.read_remote_supported_capabilities(conn_handle)
        rem_cap_event = car_node.wait_for_event(
            event_type=CsEventType.NWP_CS_READ_REMOTE_CAPS, timeout=10
        )
        if rem_cap_event:
            print_obj_data("Remote Capabilities", rem_cap_event)
        else:
            return False

        print("***** security enable *****")
        car_node.cs.security_enable(conn_handle)
        security_enable_event = car_node.wait_for_event(
            event_type=CsEventType.NWP_CS_SECURITY_ENABLE_COMPLETE, timeout=10
        )
        if security_enable_event:
            print_obj_data("Security Enable", security_enable_event)
        else:
            return False

        time.sleep(1)

        print("***** set default settings (Boosting Max TX Power to +10 dBm) *****")
        # Boost max_tx_power to 10 dBm to maximize signal power through obstacles
        car_node.cs.set_default_settings(
            conn_handle, role_enable=3, sync_anthenna_selection=1, max_tx_power=10
        )
        time.sleep(1)

        print("***** create config for connection *****")
        cs_config_params["conn_handle"] = conn_handle
        car_node.cs.create_config(cs_config_params)
        create_config_event = car_node.wait_for_event(
            event_type=CsEventType.NWP_CS_CONFIG_COMPLETE, timeout=10
        )
        if create_config_event:
            print_obj_data("Create Config", create_config_event)
        else:
            return False

    return True
"""new code ends here"""


def cs_proc_enable(
    car_node,
    conn_handle=0,
    repeat=0,
    wait_for_remote=True,
    wait_for_distance=True,
    extended_results=False,
    wait_for_raw_events=False,
    auto_config=False,
    ras_profile=False,
):
    if not auto_config:
        print("***** procedure enable *****")
        car_node.cs.procedure_enable(conn_handle=conn_handle)

    proc_enable_event = car_node.wait_for_event(
        event_type=CsEventType.NWP_CS_PROCEDURE_ENABLE_COMPLETE, timeout=10
    )
    if proc_enable_event:
        print_obj_data("Procedure Enable", proc_enable_event)

        if proc_enable_event["status"] == 0:
            for i in range(repeat):
                if wait_for_raw_events:
                    wait_results_subevents(
                        car_node, wait_for_remote, ras_profile)
                    if wait_for_distance:
                        wait_distance(car_node, extended_results)
                elif wait_for_distance:
                    return wait_distance(car_node, extended_results)
                else:
                    break


def wait_distance_extended_results_event(car_node):
    print("Waiting for NWP_CS_APP_DISTANCE_EXTENDED_RESULTS")
    distance_extended_results = car_node.wait_for_event(
        event_type=CsEventType.NWP_CS_APP_DISTANCE_EXTENDED_RESULTS, timeout=3
    )
    if distance_extended_results:
        print("Distance Extended Results")
        print("----------------------------------------------------------------------")
        for key, value in distance_extended_results.items():
            if isinstance(value, list):
                print(f"{key}: [{', '.join(str(v) for v in value)}]")
            else:
                print(f"{key}: {value}")
        print("----------------------------------------------------------------------")
    else:
        print("No NWP_CS_APP_DISTANCE_EXTENDED_RESULTS event received")
        return None


def wait_distance_event(car_node):
    print("Waiting for NWP_CS_APP_DISTANCE_RESULTS")
    distance_results = car_node.wait_for_event(
        event_type=CsEventType.NWP_CS_APP_DISTANCE_RESULTS, timeout=3
    )
    if distance_results:
        print_obj_data("Distance Results", distance_results)
        distance = distance_results["distance"]
        confidence = distance_results["confidence"]

        print(f"extracted distance: {distance} cm")
        print(f"extracted confidence: {confidence}")

        return distance, confidence


def wait_results_subevents(car_node, wait_for_remote, ras_profile=False):
    print("**** Waiting for Local Results ****")
    wait_results_subevents_data(car_node)

    if wait_for_remote:
        print("**** Waiting for Remote Results ****")
        if ras_profile:
            wait_ras_subevent_results_event(car_node)
        else:
            wait_results_subevents_data(car_node)


def wait_distance(car_node, extended_results=False):
    if extended_results:
        wait_distance_extended_results_event(car_node)
    else:
        return wait_distance_event(car_node)


def wait_results_subevents_data(car_node):
    proc_done = CsProcedureDoneStatusType.NWP_CS_PROCEDURE_ACTIVE
    subevent_done = CsSubeventDoneStatusType.NWP_CS_SUBEVENT_ACTIVE

    while proc_done == CsProcedureDoneStatusType.NWP_CS_PROCEDURE_ACTIVE:
        print("Waiting for CS_SUBEVENT_RESULTS")
        results = car_node.wait_for_event(
            event_type=CsEventType.NWP_CS_SUBEVENT_RESULTS, timeout=3
        )
        if results:
            print_obj_data("Subevent Results", results)
            proc_done = results["procedure_done_status"]
            subevent_done = results["subevent_done_status"]

        if results is None:
            print("ERROR: No CS_SUBEVENT_RESULTS arrived")
            return

        while subevent_done == CsSubeventDoneStatusType.NWP_CS_SUBEVENT_ACTIVE:
            print("Waiting for CS_SUBEVENT_RESULTS_CONTINUE")
            results = car_node.wait_for_event(
                event_type=CsEventType.NWP_CS_SUBEVENT_RESULTS_CONTINUE, timeout=3
            )
            if results:
                print_obj_data("Subevent Results Continue", results)
                proc_done = results["procedure_done_status"]
                subevent_done = results["subevent_done_status"]
            else:
                print("ERROR: No CS_SUBEVENT_RESULTS_CONTINUE arrived")
                return

            if (
                proc_done == CsProcedureDoneStatusType.NWP_CS_PROCEDURE_DONE
                and subevent_done != CsSubeventDoneStatusType.NWP_CS_SUBEVENT_DONE
            ):
                print("ERROR: Procedure done although subevent didn't!")
                return


def wait_ras_subevent_results_event(car_node):
    print("Waiting for NWP_CS_APP_RAS_SUBEVENT_RESULTS")
    decimal_fields = {
        "status",
        "conn_handle",
        "start_acl_connection_event",
        "frequency_compensation",
        "ranging_done_status",
        "subevent_done_status",
        "ranging_abort_reason",
        "subevent_abort_reason",
        "reference_power_level",
        "num_steps_reported",
        "data_len",
    }
    all_results = []
    keep_looping = True
    while keep_looping:
        ras_subevent_results = car_node.wait_for_event(
            event_type=CsEventType.NWP_CS_APP_RAS_SUBEVENT_RESULTS, timeout=3)
        if ras_subevent_results:
            print("RAS Subevent Results")
            print(
                "----------------------------------------------------------------------")
            for key, value in ras_subevent_results.items():
                if key == "data":
                    print(f"data: {value}")
                elif key in decimal_fields:
                    print(f"{key}: {value}")
                elif isinstance(value, int):
                    print(f"{key}: {hex(value)}")
                else:
                    print(f"{key}: {value}")
            print(
                "----------------------------------------------------------------------")
            all_results.append(ras_subevent_results)
            ranging_done_status = ras_subevent_results.get(
                "ranging_done_status")
            keep_looping = (ranging_done_status == 0x1)
        else:
            print("Error: No NWP_CS_APP_RAS_SUBEVENT_RESULTS event received")
            return None
    return all_results


def main():
    logging_file = get_logging_file_path()
    print(f"Logging to file: {logging_file}")

    # Initializes list for graph values (Francis's GUI additions)
    time_vals = []
    distance_vals = []
    confidence_vals = []

    # Create a BleDevice instance for the car node
    car_node = BleDevice(
        device_comport="COM3",
        logging_file=logging_file,
        logging_level=BleDeviceLoggingLevel.DEBUG,
        sync_command=False,
    )

    if car_node.initialize():
        peer_names = [
            "Key Node"
        ]
        adv_enabled = False

        cs_wait_for_distance = True
        cs_extended_results = False
        cs_wait_for_raw_events = False
        cs_wait_for_remote = True
        cs_auto_config = False
        cs_ras_profile = False

        car_node.cs.set_default_antenna(0)

        if adv_enabled:
            print("Start Advertising")
            car_node.peripheral.start_advertising(0, 0, 0)
            car_node.peripheral.wait_for_event(
                event_type=PeripheralEventType.NWP_ADV_START_AFTER_ENABLE, timeout=30
            )

        print("Start Scanning for peer devices")
        connections = search_and_connect_to_peer(car_node, peer_names)
        if not connections:
            print("ERROR: No peer devices found!")
            return

        print(f"Connections: {connections}")

        for conn_handle in connections.values():
            status = start_cs_config(car_node, conn_handle)
            if status == False:
                print(
                    f"ERROR: CS Config with connection {conn_handle} Failed!")
                return

        # Francis's live graph setup
        plt.ion()

        fig, (ax_distance, ax_confidence) = plt.subplots(2, 1, sharex=True)
        distance_line, = ax_distance.plot(time_vals, distance_vals)
        confidence_line, = ax_confidence.plot(time_vals, confidence_vals)
        ax_distance.set_ylabel("Distance (cm)")
        ax_distance.set_title("Channel Sounding Distance")

        ax_confidence.set_ylabel("Confidence")
        ax_confidence.set_xlabel("Time (sec)")
        ax_confidence.set_ylim(0, 105)

        start_time = time.monotonic()
        for i in range(170):
            for conn_handle in connections.values():
                cs_set_procedure_params["conn_handle"] = conn_handle

                if not cs_auto_config:
                    car_node.cs.set_procedure_params(cs_set_procedure_params)
                time.sleep(0.2)

                result = cs_proc_enable(
                    car_node,
                    conn_handle,
                    cs_set_procedure_params["max_procedure_count"],
                    wait_for_remote=cs_wait_for_remote,
                    wait_for_distance=cs_wait_for_distance,
                    extended_results=cs_extended_results,
                    wait_for_raw_events=cs_wait_for_raw_events,
                    auto_config=cs_auto_config,
                    ras_profile=cs_ras_profile,
                )
                if result:
                    distance, confidence = result
                    
                    """ new code here that gets rid of high outliers """
                    # Filters out corrupt 0xFFFFFFFF / 4*10^9 cm error spikes during signal loss
                    MAX_VALID_DISTANCE_CM = 10000

                    if distance < MAX_VALID_DISTANCE_CM and confidence > 0:
                        elapsed_time = time.monotonic() - start_time
                        time_vals.append(elapsed_time)
                        distance_vals.append(distance)
                        confidence_vals.append(confidence)

                        # Francis's dynamic line update and axis rescale logic
                        distance_line.set_data(time_vals, distance_vals)
                        confidence_line.set_data(time_vals, confidence_vals)

                        ax_distance.relim()
                        ax_distance.autoscale_view()

                        ax_confidence.relim()

                        print(
                            f"MAIN -> Distance: {distance} cm, Confidence: {confidence}")
                    else:
                        print(
                            f"MAIN -> Ignored invalid spike reading: Distance={distance} cm, Confidence={confidence}")
                    """ new code ends here """

                # Keeps the Matplotlib GUI responsive
                plt.pause(0.01)

    car_node.done()

    plt.ioff()
    plt.show()

    print("Example ended")

    exit(0)


if __name__ == "__main__":
    main()