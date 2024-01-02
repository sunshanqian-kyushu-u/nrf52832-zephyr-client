#include <zephyr/kernel.h>

/* BT推荐引入的库 */
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

/* 控制台库 */
#include <zephyr/sys/printk.h>

#include <dk_buttons_and_leds.h>

static struct bt_conn *default_conn;

static void start_scan(void); // 声明

static const uint8_t MACADDRESS[BT_ADDR_SIZE] = {0x52, 0x42, 0x54, 0x00, 0x00, 0x00};

static void device_found_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
		struct net_buf_simple *ad) {
	char addr_str[BT_ADDR_LE_STR_LEN];
	int err;

	if (default_conn) {
		return;
	}

	/* 可扫描可连接、可直接连接 */
	if (type != BT_GAP_ADV_TYPE_ADV_IND &&
	        type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return;
	}

    /* 将第一个参数中的地址传给第二个参数，这里的地址是MAC地址 */
	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	printk("Device found: %s (RSSI %d)\n", addr_str, rssi);

    /* 在这里判断是否是目标服务器 */
	if (strcmp(MACADDRESS, addr->a.val)) { // MAC地址不匹配
		return;
	}

    /* 搜索到了服务器后停止扫描 */
	if (bt_le_scan_stop()) {
		return;
	}

    /* 进行连接，应该是写入到default_conn */
	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
			BT_LE_CONN_PARAM_DEFAULT, &default_conn);
	if (err) {
		printk("Create conn to %s failed (%d)\n", addr_str, err);
		start_scan();
	}
}

static void start_scan(void) {
	int err;

	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found_cb);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}

	printk("Scanning successfully started\n");
}

static void conn_disconnected(struct bt_conn *conn, uint8_t reason) {
	char addr[BT_ADDR_LE_STR_LEN];

	if (conn != default_conn) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s (reason 0x%02x)\n", addr, reason);

	bt_conn_unref(default_conn);
	default_conn = NULL;

	start_scan();

	// printk("Disconnected\n");
}

/* LED状态特征 */
#define BT_UUID_LED_STATUS_CHARACTERISTIC_VAL \
	BT_UUID_128_ENCODE(0x12345679, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)
#define BT_UUID_LED_STATUS_CHARACTERISTIC \
	BT_UUID_DECLARE_128(BT_UUID_LED_STATUS_CHARACTERISTIC_VAL)

/* LED控制特征 */
#define BT_UUID_LED_CONTROL_CHARACTERISTIC_VAL \
	BT_UUID_128_ENCODE(0x1234567a, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)
#define BT_UUID_LED_CONTROL_CHARACTERISTIC \
	BT_UUID_DECLARE_128(BT_UUID_LED_CONTROL_CHARACTERISTIC_VAL)

static struct bt_uuid_128 uuid_128 = BT_UUID_INIT_128(0);
static struct bt_uuid_16 uuid_16 = BT_UUID_INIT_16(0);

static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params subscribe_params;

static uint16_t led_status_characteristic_value_handle;
static uint16_t led_control_characteristic_value_handle;

/* 通知回调函数 */
static uint8_t notify_func(struct bt_conn *conn,
		struct bt_gatt_subscribe_params *params,
		const void *data, uint16_t length) {
	if (!data) {
		printk("[UNSUBSCRIBED]\n");
		params->value_handle = 0U;
		return BT_GATT_ITER_STOP;
	}

	printk("[NOTIFICATION] data %p length %u\n", data, length);

	u_int8_t temp_notification_data[length + 1];
	memcpy(temp_notification_data, data, length);
	temp_notification_data[length] = 0x00;
	dk_set_leds(temp_notification_data[0]);

	return BT_GATT_ITER_CONTINUE;
}

/***
 * 需要说明的是当满足discover_params条件时会触发这个回调函数
 * 有可能会有很多个，所以需要在得到了想要的handle后return
 ***/
static uint8_t discover_func(struct bt_conn *conn,
		const struct bt_gatt_attr *attr,
		struct bt_gatt_discover_params *params) {

	int err;

	if (!attr) {
		printk("Discover complete\n");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	printk("Find handle %u\n", attr->handle);

	if (!bt_uuid_cmp(discover_params.uuid, 
			BT_UUID_LED_STATUS_CHARACTERISTIC)) {
		// 匹配到led status characteristic declaration attribute
		printk("led status characteristic declaration attribute handle %u\n", 
				attr->handle); //需要注意这里输出的是十进制
		led_status_characteristic_value_handle = bt_gatt_attr_value_handle(attr);
		printk("led status characteristic value attribute handle %u\n", 
				led_status_characteristic_value_handle);

		memcpy(&uuid_16, BT_UUID_GATT_CCC, sizeof(uuid_16));
		discover_params.uuid = &uuid_16.uuid;
		discover_params.start_handle = attr->handle + 2; // +1是c_v
		discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
		subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}

	} else if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_GATT_CCC)) {
		// 匹配到characteristic descriptor attribute
		printk("led status characteristic descriptor attribute handle %u\n", 
				attr->handle);
		subscribe_params.notify = notify_func;
		subscribe_params.value = BT_GATT_CCC_NOTIFY;
		subscribe_params.ccc_handle = attr->handle;

		err = bt_gatt_subscribe(conn, &subscribe_params);
		if (err && err != -EALREADY) {
			printk("Subscribe failed (err %d)\n", err);
		} else {
			printk("Subscribe\n");
		}

		memcpy(&uuid_128, BT_UUID_LED_CONTROL_CHARACTERISTIC, sizeof(uuid_128));
		discover_params.uuid = &uuid_128.uuid;
		discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid, 
			BT_UUID_LED_CONTROL_CHARACTERISTIC)) {
		// 匹配到led control characteristic declaration attribute
		printk("led control characteristic declaration attribute handle %u\n", 
				attr->handle);
		led_control_characteristic_value_handle = bt_gatt_attr_value_handle(attr);
		printk("led control characteristic value attribute handle %u\n", 
				led_control_characteristic_value_handle);
		return BT_GATT_ITER_STOP;
	}
	return BT_GATT_ITER_STOP;
}

static void conn_connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

    /* 将地址保存到addr中 */
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		printk("Failed to connect to %s (%u)\n", addr, err);

		bt_conn_unref(default_conn);
		default_conn = NULL;

		start_scan();
		return;
	}

	if (conn != default_conn) {
		return;
	}

	printk("Connected: %s\n", addr);

	/*** 
	 * 建立连接之后需要将服务端的灯的状态读取回来，另外还需要发送灯控命令
	 * 这些都是通过handle完成的
	 * 要获取led status的handle，就要取得其characteristic
	 * 然后调用bt_gatt_attr_value_handle获取handle
	 ***/
	int discover_err;

	memcpy(&uuid_128, BT_UUID_LED_STATUS_CHARACTERISTIC, sizeof(uuid_128));
	discover_params.uuid = &uuid_128.uuid;
	discover_params.func = discover_func;
	discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

	discover_err = bt_gatt_discover(default_conn, &discover_params);
	if (discover_err) {
		printk("Discover failed(err %d)\n", discover_err);
		return;
	}
}

/* 该例程中直接用宏而不是使用bt_conn_cb_register注册连接回调函数 */
BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = conn_connected,
	.disconnected = conn_disconnected,
};

static void button_handler(uint32_t button_state, uint32_t has_changed)
{
	int err;

	static uint8_t data1[1] = {0x01};
	static uint8_t data2[1] = {0x02};
	static uint8_t data3[1] = {0x04};
	static uint8_t data4[1] = {0x08};

	if (has_changed & button_state) {
		switch (has_changed) {
			case DK_BTN1_MSK:
				err = bt_gatt_write_without_response(default_conn, 
						led_control_characteristic_value_handle, 
						data1, sizeof(data1), false);
				if (err) {
					printk("send cmd failed (err %d)\n", err);
				}
				break;
			case DK_BTN2_MSK:
				err = bt_gatt_write_without_response(default_conn, 
						led_control_characteristic_value_handle, 
						data2, sizeof(data2), false);
				if (err) {
					printk("send cmd failed (err %d)\n", err);
				}
				break;
			case DK_BTN3_MSK:
				err = bt_gatt_write_without_response(default_conn, 
						led_control_characteristic_value_handle, 
						data3, sizeof(data3), false);
				if (err) {
					printk("send cmd failed (err %d)\n", err);
				}
				break;
			case DK_BTN4_MSK:
				err = bt_gatt_write_without_response(default_conn, 
						led_control_characteristic_value_handle, 
						data4, sizeof(data4), false);
				if (err) {
					printk("send cmd failed (err %d)\n", err);
				}
				break;
			default:
				break;
		}
	}
}

int main(void)
{
    int err;

    err = dk_leds_init();
	if (err) {
		printk("LEDs init failed (err %d)\n", err);
		return 0;
	}

    dk_set_leds(DK_NO_LEDS_MSK); // 关掉所有LED
	printk("LEDs init succeed\n");

    err = dk_buttons_init(button_handler);
	if (err) {
		printk("Button init failed (err %d)\n", err);
		return 0;
	}

	printk("Button init succeed\n");

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

    printk("Bluetooth initialized\n");

    start_scan();

    return 0;
}


