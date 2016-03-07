#ifdef PARTICLE_LOCAL_BUILD
#include "swd.h"
#else
#pragma SPARK_NO_PREPROCESSOR
#include "swd/swd.h"
#endif
SYSTEM_MODE(MANUAL);

extern const uint8_t bootloader_platform_6_bin[];
extern const uint32_t bootloader_platform_6_bin_len;
extern const uint8_t bootloader_platform_8_bin[];
extern const uint32_t bootloader_platform_8_bin_len;
extern const uint8_t bootloader_platform_10_bin[];
extern const uint32_t bootloader_platform_10_bin_len;

struct PlatformBootloader
{
	uint16_t platform_id;
	const char* article;
	const char* name;
	const uint8_t* data;
	uint32_t size;
};

class Fixit
{
	Print& out;
	Stream& in;

	SWDProtocolSupport<DigitalPinSWDDriver> swd;
	DebugPort debugPort;
	STM32F205 target;

	const PlatformBootloader bootloaders[3] = {
			{ 6, "a","Photon", bootloader_platform_6_bin, bootloader_platform_6_bin_len },
			{ 8, "a", "P1", bootloader_platform_8_bin, bootloader_platform_8_bin_len },
			{ 10, "an", "Electron", bootloader_platform_10_bin, bootloader_platform_10_bin_len }
	};
	const int8_t platform_count = arraySize(bootloaders);

public:

	template<typename T>Fixit(T& t, pin_t swdclk, pin_t swdio) : Fixit(t, t, swdclk, swdio) {}

	Fixit(Print& out_, Stream& in_, pin_t swdck, pin_t swdio)
	: out(out_), in(in_), swd(out_, swdck, swdio),
			debugPort(swd, out), target(debugPort) {}

	void setup()
	{
		RGB.control(true);
		RGB.color(0,0,255);
		while (!Serial.available()) {}
		while (Serial.available()) Serial.read();
	}

	uint8_t option(const char* msg, std::initializer_list<const char*> opts, int def=-1, bool listOpts=true)
	{
		if (def==-1)
			def = opts.size();
		return option(msg, opts.begin(), opts.size(), def, listOpts);
	}

	uint8_t option(const char* msg, const char* const* opts, uint8_t count, uint8_t def, bool listOpts)
	{
		uint8_t result = count;

		while (result==count) {
			out.print(msg);
			if (listOpts) {
				out.print(' ');
				out.print('[');
				for (int i=0; i<count; i++)
				{
					if (i) out.print(",");
					out.print(opts[i]);
				}
				out.print(']');
			}
			out.print(':');
			char input[20];

			serialReadLine(&in, input, 20, 0);
			out.println();
			if (!*input)					// no input given, use default
				result = def;
			else
				for (uint8_t i=0; i<count; i++)
				{
					if (!strcasecmp(opts[i], input))
						result = i;
				}

			if (result==count)
				out.println("Not sure about that.");
		}
		return result;
	}

	void loop()
	{
		out.println("Bootloader fixerupper, v0.0");
		out.println("This utility fixes up the bootloader on another device. ");
		out.println("Before running this, be sure you have correctly wired the programmer and target devices.");
		out.println();
		out.println("WARNING: Do not use this with a target device that is showing LED activity.");
		out.println("LED activity on the target device means the bootloader is functioning and any problems with");
		out.println("the device can be fixed using particle-cli.");
		out.println();
		out.println("WARNING^2: Please read the message above!");
		out.println();


		for (;;)
		{
			if (option("Do you have a dead device and wish to continue?", { "y", "N" }, 1)==0)
			{
				SWDResult error = startFlash();
				if (failed(error))
				{
					handleError(error);
				}
				else
				{
					break;
				}
			}
			else
				break;
		}
		for (;;) { advancedMenu(); }
	}

	int8_t platformIndex(uint16_t platformID)
	{
		int8_t index = -1;
		for (int8_t i=0; i<platform_count; i++)
		{
			if (bootloaders[i].platform_id==platformID)
				index = i;
		}
		return index;
	}

	SWDResult startFlash()
	{
		RGB.color(255,64,0);
		CHECK_SUCCESS(doDetectDevice());
		uint16_t platformID;
		CHECK_SUCCESS(readPlatformID(platformID));
		int8_t idx = platformIndex(platformID);
		if (idx>=0)
		{
			out.printlnf("It looks like you have %s %s.", bootloaders[idx].article, bootloaders[idx].name);
			if (option("Is this correct?", { "Y", "n" }, 0)==1)
			{
				idx = -1;
				out.println("nope? No problem!");
			}
		}

		if (idx<0)
		{
			char msg[120];
			char devices[60];
			int offset = 0;
			for (int i=0; i<platform_count; i++)
			{
				offset += sprintf(devices+offset, "%d=%s, ", i, bootloaders[i].name);
			}
			sprintf(msg, "What's the type of the device that you want to revive? [%sX=Exit]", devices);
			idx = option(msg, { "0","1", "2", "X" }, -1, false);
			if (idx==platform_count)
			{
				idx = -1;
				out.println("Alrighty. See ya around!");
			}
		}

		if (idx>=0)
		{
			out.printlnf("Gotcha...let's rock this! Flashing %s bootloader...", bootloaders[idx].name);
			CHECK_SUCCESS(flashPlatform(bootloaders[idx]));
			out.println("Aw yeah! Device successfully flashed. You should now see some LED activity on the device!");
			RGB.color(0,255,0);
		}
		else
		{
			RGB.color(0,0,255);
		}
		return SWDResult::Ok;
	}

	SWDResult flashPlatform(const PlatformBootloader& bl)
	{
		return doFlashDevice(bl);
	}

	/**
	 * Attempts to identify the platform from the system firmware platform ID.
	 */
	SWDResult readPlatformID(uint16_t& id)
	{
		uint32_t value1, value2, value3;
		id = 0xFFFF;
		CHECK_SUCCESS(target.read(0x8020000+388+12, value1));
		CHECK_SUCCESS(target.read(0x8020000+388+12, value1));
		CHECK_SUCCESS(target.read(0x8040000+388+12, value2));
		CHECK_SUCCESS(target.read(0x8060000+388+12, value3));
		value1 &= 0xFFFF;
		value2 &= 0xFFFF;
		value3 &= 0xFFFF;
		if (value1==6 && value3==6)
			id = 6;
		else if (value1==8 && value3==8)
			id = 8;
		else if (value1==10 && value2==10)
			id = 10;

		return SWDResult::Ok;
	}

	void advancedMenu()
	{
		if (in.available())
		{
			char c = in.read();
			if (c=='!')
			{
				out.println("Guru options.");
				out.println("b: dump start of bootloader");
				out.println("1: dump module info from module 1");
				out.println("0: set RDP level 0");
				out.println("E: erase bootloader");

				while (!in.available());
				c = in.read();
				SWDResult result = doDetectDevice();
				switch (c)
				{
				case 'b': result = dumpMemory(target.BOOTLOADER_ADDRESS, 256); break;
				case '1': result = dumpModule(0x8020000); break;
				case '0': result = resetRDP(); break;
				case 'E': result = eraseBootloader(); break;
				default:
					out.println("That gets us nowehere.");
				}
				if (failed(result))
					printResult(result);
			}
		}
	}


	const char* resultToString(SWDResult result)
	{
		switch (result) {
		case SWDResult::Ok: return "Ok";
		case SWDResult::ProtocolError: return "SWD Protocol error";
		case SWDResult::WaitError: return "SWD Wait error";
		case SWDResult::ParityError: return "SWD Parity error";
		case SWDResult::FaultError: return "SWD Fault error";
		case SWDResult::InitializationError: return "SWD Initialization error";
		case SWDResult::MemAPInitializationError: return "MDM_AP Initialization error";
		case SWDResult::DeviceNotPresent: return "Device not detected.";
		case SWDResult::UnknownAckError3: return "Unknown ACK (3) received.";
		case SWDResult::UnknownAckError5: return "Unknown ACK (5) received.";
		case SWDResult::UnknownAckError6: return "Unknown ACK (6) received.";
		case SWDResult::OperationError: return "Operation Error";
		case SWDResult::FlashError: return "Flash Error";
		default:
			return "Mysterious error";
		}
	}

	void printResult(SWDResult result)
	{
		const char* s = resultToString(result);
		out.print(s);
	}

	SWDResult doDetectDevice()
	{
		CHECK_SUCCESS(target.initialize());
		CHECK_SUCCESS(target.halt());
		CHECK_SUCCESS(target.reset());
		return SWDResult::Ok;
	}

	void handleError(SWDResult result)
	{
		printResult(result);
		out.println();
		out.println("Please double check the wiring, then try again.");
	}

	SWDResult doFlashDevice(const PlatformBootloader& bl, bool eraseOnly=false)
	{
		CHECK_SUCCESS(doDetectDevice());
		CHECK_SUCCESS(target.flashBootloader(bl.data, bl.size, eraseOnly));
		return SWDResult::Ok;
	}

	SWDResult readMemory(uint32_t* buf, size_t len, uint32_t address)
	{
		while (len-->0)
		{
			uint32_t value;
			CHECK_SUCCESS(target.read(address+=4, value));
			*buf++ = value;
		}
		return SWDResult::Ok;
	}

	SWDResult dumpMemory(uint32_t address, size_t len)
	{
		const size_t block_size = 32;
		uint8_t data[32];
		memset(data, 0, block_size);
		while (len)
		{
			const size_t block = len>32 ? 32 : len;
			CHECK_SUCCESS(readMemory((uint32_t*)data, block>>2, address));
			out.printf("%08x: ", address);
			for (uint8_t i=0; i<block; i++) {
				if (i && (i%4==0))
					out.print(' ');
				out.printf("%02x", data[i]);
			}
			out.println();
			len -= block;
			address += block;
		}
		return SWDResult::Ok;
	}

	SWDResult dumpModule(uint32_t address)
	{
		return dumpMemory(address+384, 24);
	}

	SWDResult resetRDP()
	{
		uint32_t opts;
		CHECK_SUCCESS(target.flashOptionsUnlock());
		CHECK_SUCCESS(target.read(target.FLASH_OPTCR, opts));
		if (target.isRDPLevel0(opts))
		{
			out.println("Device already at RDP Level 0");
		}
		else
		{
			out.println("Setting RDP level 0");
			CHECK_SUCCESS(target.setRDPLevel0(opts));
		}
		return SWDResult::Ok;
	}

	SWDResult eraseBootloader()
	{
		out.println("Erasing bootloader...");
		CHECK_SUCCESS(target.eraseBootloader());
		out.println("Bootloader erased.");
		return SWDResult::Ok;
	}
};


Fixit fixit(Serial, D6, D7);


void setup()
{
	Serial.begin(9600);
	fixit.setup();
}

void loop()
{
	fixit.loop();
}
