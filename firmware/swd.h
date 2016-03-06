
#include "Particle.h"
#include <functional>
#include <algorithm>

/*
 * REFERENCES
 *
 * EMT32 SWD Programming
 * https://www.silabs.com/Support%20Documents/TechnicalDocs/AN0062.pdf
 * Gives an overview of the SWD protocol, Debug and Access Ports.
 *
 * Cortex M3 Technical Reference Manual (for register addresses)
 * http://infocenter.arm.com/help/topic/com.arm.doc.ddi0337h/DDI0337H_cortex_m3_r2p0_trm.pdf
 *
 * ARMv7-M Architecture Reference Manual (for register behavior)
 * https://web.eecs.umich.edu/~prabal/teaching/eecs373-f10/readings/ARMv7-M_ARM.pdf
 *
 * STM32 Flash Programming Manual
 * http://www.st.com/web/en/resource/technical/document/programming_manual/CD00233952.pdf
 *
 * ARM debug interface v5 Architecture Specification
 * http://hackipedia.org/Hardware/CPU/ARM/pdf,%20Cortex/IHI0031A_ARM_debug_interface_v5.pdf
 *
 * LibSWD
 * https://fedcsis.org/proceedings/2012/pliks/279.pdf
 *
 * Overview of SWD
 * https://www.lpcware.com/content/blog/introduction-cortex-serial-wire-debugging-part-one
 *
 *
 */

/**
 * SWD Acknowledgement values. (3-bits read LSB first.)
 */
enum class Ack : uint8_t
{
	// "Serial Wire Debug and the CoreSightTM Debug and Trace Architecture"

	// Note: ACK is transmitted LSB first, so patterns
	// below are reversed comparing to ARMDIv5 diagrams, but matches
	// values used in text (sec. 5.4.2 and on)
	OK = 0b001,
	WAIT = 0b010,
	FAULT = 0b100,
	// Additional code not defined in ARMDIv5 - when target not present/
	// doesn't respond, line is pulled high and that's what we read
	NOTPRESENT = 0b111,
};

/**
 * Port select bits in the request byte.
 */
enum class Port : uint8_t {
	// # Same note on bit order applies. The debug port registers also fit into this scheme.
	AP = 0x02,
	DP = 0x00
};

/**
 * Direction selection bits in the request byte.
 */
enum class Direction : uint8_t {
	READ = 0x04,
	WRITE = 0x00
};

/**
 * Parity bit in the request byte. Set if the request byte contains an odd number of 1 bits.
 */
enum class Parity : uint8_t {
	PARITY = 0x20
};

/**
 * The size of a single data transfer over SWD.
 */
typedef uint32_t swd_data_t;


enum class SWDResult
{

	/**
	 * The transaction was successful.
	 */
	Ok,

	/**
	 * The target is busy - the operation should be retried again later.
	 */
	WaitError,

	/**
	 * One or more sticky error bits are set.
	 */
	FaultError,

	/**
	 * Unable to communicate with the target. The debugger should first try reading the IDCODE.
	 * If that fails, the debugger should reset the line then retry reading the IDCODE register.
	 */
	ProtocolError,

	/**
	 * The data received from the target failed the parity check.
	 */
	ParityError,

	/**
	 * Unable to initialize access to the Debug Port
	 */
	InitializationError,

	/**
	 * Unable to initialize access to the Debug Port
	 */
	MemAPInitializationError,

	/**
	 * Couldn't detect the presence of the device via the ACK response.
	 */
	DeviceNotPresent,

	/**
	 *
	 */
	UnknownAckError5, 	UnknownAckError6,  UnknownAckError3,

	/**
	 * The target device didn't perform as expected.
	 */
	OperationError,

	/**
	 * Couldn't perform a flash operation within a given timeout.
	 */
	FlashError
};

template <typename E>
constexpr typename std::underlying_type<E>::type to_underlying(E e) {
    return static_cast<typename std::underlying_type<E>::type>(e);
}


/**
 * Describes the SWD interface via its read/write transactions.
 */
class SWDProtocol
{

public:

    virtual SWDResult initialize()=0;

    /**
     * Reads a 32-bit value from a target port register. The returned result indicates if the read was successful or not.
     *
     * @param port 	The port to read from
     * @param reg	The port register to read from, expressed as the register index * 8 (so that it can be inserted directly in the request byte.)
     * @param result	 The 32-bits of data retrieved from the device. Only valid if SWDResult::OK is returned.
     */
    virtual SWDResult read(Port port, uint8_t reg, swd_data_t& result)=0;

    /**
     * Writes a 32-bit value to a target port register.
     *
     * @param port 	The port to write to
     * @param reg	The port register to write to, expressed as the register index * 8 (so that it can be inserted directly in the request byte.)
     * @param data	The 32-bits of data to write to the port register.
     * @return SWDResult::OK if the data was written successfully.
     */
    virtual SWDResult write(Port p, uint8_t reg, swd_data_t data)=0;

};

inline bool failed(SWDResult result) { return result!=SWDResult::Ok; }

/**
 * Provides the wire protocol independently from how the SWDIO and SWDCK lines are manipulated.
 *
 * Data is always transmitted LSB first over the wire.
 * When driver changes from host to target, a one cycle turnaround period is inserted.
 * Operations on a WAIT response should be retried
 * target does not send a reply when there's a parity error. host must backoff, and read the IDCODE register.
 * if that doesn't work then perform a line reset and retry.
*/
template<typename SWDDriver>
class SWDProtocolSupport : public SWDProtocol
{
	Print& out;
	SWDDriver driver;

public:
	template<typename ...Args> SWDProtocolSupport(Print& output, Args... args) : out(output), driver(args...) {}


	/**
	 * Initializes the driver and the device to begin accepting the SWD protocol.
	 */
	SWDResult initialize() override
    {
		driver.initialize();
		lineReset();
		writeBits(0xe79e, 16);
		lineReset();
		writeBits(0, 2);
		return SWDResult::Ok;
    }

	/**
	 * Read a register from a given port.
	 */
    SWDResult read(Port port, uint8_t reg, swd_data_t& result)
    {
    		uint8_t request = buildRequest(port, reg, Direction::READ);

    		SWDResult error = SWDResult::WaitError;
    		while (error==SWDResult::WaitError)
    		{
			error = sendRequest(request);
			if (!failed(error))
			{
				error = readPayload(result);
			}
			driver.idle();
    		}
    		return error;
    }

    SWDResult write(Port port, uint8_t reg, swd_data_t data)
    {
		uint8_t request = buildRequest(port, reg, Direction::WRITE);
		SWDResult error = SWDResult::WaitError;
		while (error==SWDResult::WaitError)
		{
			error = sendRequest(request);
			if (!failed(error))
			{
				writePayload(data);
			}
			driver.idle();
		}
		return error;
    }

protected:

    void resetSWD()
    {
        this->lineReset();
    }

    /**
     * Reset the SWD protocol.
     */
	void lineReset()
	{
		// Hold the sdwio high for at least 50 cycles
		// this should porobably be part of the driver
		this->sendByteRepeat(0xFF, 8);
	    driver.idle();
	}


    void sendByteRepeat(uint8_t value, uint8_t count)
	{
		while (count-->0)
			writeBits(value, 8);
	}

    /**
     * Determines the number of bits set in a given byte
     */
    static uint8_t bitCount(uint32_t data)
    {
    		return __builtin_popcount(data);
	}

    /**
     * Sends the SWD request to the target and retrieves the ack code.
     */
    SWDResult sendRequest(uint8_t request)
    {
    		this->writeBits(0, 1);
    		this->writeBits(request, 8);
    		return readAck();
    }

    constexpr inline static uint8_t as_int(Ack ack)
    {
    		return static_cast<uint8_t>(ack);
    }

    /**
     * Retrieves the 3 acknowledgement bits and converts these to a result code.
     */
    SWDResult readAck()
    {
    		uint8_t ack = this->readBits(3);
    		SWDResult error = SWDResult::ProtocolError;
    		switch (ack) {
			case as_int(Ack::OK):
				error = SWDResult::Ok;
				break;
			case as_int(Ack::WAIT):
				error = SWDResult::WaitError;
				break;
			case as_int(Ack::FAULT):
				error = SWDResult::FaultError;
				break;
			case as_int(Ack::NOTPRESENT):
				error = SWDResult::DeviceNotPresent;
				break;
			case 5:
				error = SWDResult::UnknownAckError5;
				break;
			case 6:
				error = SWDResult::UnknownAckError6;
				break;
			case 3:
				error = SWDResult::UnknownAckError3;
				break;
    		}

		// todo - If the Overrun Detect bit in the DP CTRL/STAT Register is set to 1, then a data transfer
		// phase is required on all responses, including WAIT and FAULT.
    		return error;
    }

    /**
     * Reads 33 bits from the device.
     */
    SWDResult readPayload(uint32_t& result)
    {
    		result = readBits(32);
    		uint8_t parity = readBits(1);		// party
    		uint8_t count = bitCount(result) + (parity&1);
    		//Serial.printlnf("got result %x (%d), count=%d, parity %d, error=%d", result, bitCount(result), count, parity, count&1);
    		SWDResult error = SWDResult::Ok;
    		if (count & 1) {  // should be even parity
    			error = SWDResult::ParityError;
    		}
    		return error;
    }

    void writePayload(uint32_t data)
    {
    		writeBits(data, 32);
    		// write the even parity bit
    		writeBits(bitCount(data), 1);
    }

    /**
     * Builds the SWD request byte that starts each transaction.
     * @param port the port to access either AP or DP
     * @param reg The register to read. This must be a multiple of 8 between 0 and 24 inclusive.
     */
    uint8_t buildRequest(Port port, uint8_t reg, Direction direction)
    {
    		uint8_t result = 0;
    		result |= static_cast<int>(port);
    		result |= static_cast<int>(reg);
    		result |= static_cast<int>(direction);
    		if (bitCount(result) & 1)			// odd bit count so add party bit to make even
    			result |= static_cast<int>(Parity::PARITY);
    		result |= 0x81;						// park/stop/start bits
    		return result;
    }

    /**
     * Sends a given number of bytes to the target.
     */
    void sendBytes(const uint8_t* data, size_t count)
    {
    		while (count-->0)
    			sendByte(*data++);
    }

    void sendByte(uint8_t data)
    {
    		writeBits(data, 8);
    }

    /**
     * Sends an array of bytes. The bytes are written LSB first from lowest to highest index.
     */
	template<size_t N> void sendBytes(const uint8_t data[N])
	{
		sendBytes(data, N);
	}

    /**
     * Reads a number of bits from the SWD interface.
     */
     inline uint32_t readBits(uint8_t count)
     {
    	 	 uint32_t bits = driver.readBits(count);
		 //out.printlnf("reading: %x (%d)", bits, count);

    	 	 return bits;
     }

    /**
     * Writes the given data one bit at a time, LSB first.
     */
    void writeBits(uint32_t bits, uint8_t count)
    {
    		//out.printlnf("writing: %x (%d)", bits, count);
    		driver.writeBits(bits, count);
    }

    void writeBitsArray(const uint8_t* data, uint8_t count)
    {
    		while (count>=8)
    		{
    			writeBits(*data++, 8);
    			count -= 8;
    		}
		writeBits(*data++, count);
    }
};

/**
 * Drives the SWD wire protocol using GPIOs.
 */
class DigitalPinSWDDriver
{
	pin_t swdck;
	pin_t swdio;

	Direction direction;
	bool modeSet = false;
	uint8_t dataLineLevel = HIGH;

	inline void setMode(Direction d)
	{
		pinMode(swdio, d==Direction::WRITE ? OUTPUT : INPUT);
		// debugging
		digitalWrite(A2, d==Direction::READ ? HIGH : LOW);
	}

	inline void setDataLine(uint8_t level)
	{
		// without this, I see small 45ns glitches where the line goes low in a run of setting only HIGH values.
		if (level!=dataLineLevel)
		{
			dataLineLevel = level;
			digitalWriteFast(swdio, level);
		}
	}

	inline void setDirection(Direction d)
	{
		if (d!=direction)
		{
			modeSet = false;
			direction = d;
			// if we are changing direction, then add a turnaround period
			pinMode(swdio, INPUT);
			turnaround();
			updateMode();
		}
	}

	inline void clock(uint8_t state)
	{
		digitalWriteFast(swdck, state);
	}

	inline void updateMode()
	{
		if (!modeSet)
		{
			modeSet = true;
			setMode(direction);
		}
	}

	void turnaround()
	{
		strobeClock();
	}

	void strobeClock()
	{
		clock(HIGH);
		clockInterval();
		clock(LOW);
		clockInterval();
	}

	void writeBit(uint8_t state)
	{
		clock(HIGH);
		clockInterval();
		clock(LOW);
		setDataLine(state);
		clockInterval();
	}

	uint8_t readBit()
	{
		clock(HIGH);
		clockInterval();
		clock(LOW);
		uint8_t result = digitalRead(swdio) ? 1 : 0;
		clockInterval();
		return result;
	}

    void clockInterval()
    {
        delayMicroseconds(1);
    }


public:
    /**
     * Creates a new SWD Driver on two GPIO pins.
     *
     * @param swdck_	The pin that is connected to the SWDCK input pin on the target
     * @param swdio_ The pin that is connected to the SWDIO bi-directional pin on the target
     */
	DigitalPinSWDDriver(pin_t swdck_, pin_t swdio_)
		: swdck(swdck_), swdio(swdio_), direction(Direction::WRITE)
	{
	}

	void initialize()
	{
		pinMode(swdck, OUTPUT);		// clock is output
		pinMode(swdio, OUTPUT);		// io is output
		clock(LOW);
		setDataLine(LOW);			// idle
		pinMode(A2, OUTPUT);
		digitalWrite(A2, LOW);
		direction = Direction::READ;
		modeSet = false;
		updateMode();
	}

	/**
	 * Reads a given number of bits from the SWD channel.
	 * @param count The number of bits to read. Up to 32 bits can be read in one call.
	 */
    uint32_t readBits(uint8_t count)
    {
		setDirection(Direction::READ);
		// SWD protocol requires the LSB first
    		uint32_t result = 0;
    		uint8_t remaining = count;
    		while (remaining-->0)
    		{
    			result >>= 1;
    			if (readBit())
    				result |= 1<<31;
    		}
    		result >>= (32-count);
    		return result;
    }

    /**
     * Writes a number of bits to the SWD channel.
     * @param data	The bits to write. The bits are written from the LSB and towards the MSB.
     * @param count	The number of bits to write.
     */
    void writeBits(uint32_t data, uint8_t count)
    {
		setDirection(Direction::WRITE);
    		while (count-->0)
    		{
    			writeBit(data & 1);
    			data >>= 1;
    		}
    }

    void idle()
    {
    }
};


#define CHECK_SUCCESS(x) \
	do { SWDResult __result = x; if (failed(__result)) return __result; } while (0);


class DebugPort
{
	/**
	 * The Debug Port registers that can be read
	 */
	enum class Read : uint8_t
	{
		IDCODE = 0 << 3,     // Read
		CTRL_STAT = 1 << 3,  // Read/Write, CTRLSEL=0
		WCR = 1 << 3,        // Read/Write, CTRLSEL=1
		RESEND = 2 << 3,     // Read
		READBUFF = 3 << 3   // Read
	};

	/**
	 * The Debug Port registers that can be written
	 */
	enum class Write : uint8_t
	{
		ABORT = 0 << 3,      // Write
		CTRL_STAT = 1 << 3,  // Read/Write, CTRLSEL=0
		WCR = 1 << 3,        // Read/Write, CTRLSEL=1
		SELECT = 2 << 3,     // Write
		ROUTESEL = 3 << 3,   // Write, reserved
	};

    static constexpr uint32_t ID_CODES[3] = {
        0x2BA01477,  // STM32
    };

    SWDProtocol& swd;
    Print& output;

    /**
     * Maintains the current AP register and the current bank saved to the select register.
     */
    uint8_t currentAP;
    uint8_t currentBank;

public:

    DebugPort(SWDProtocol& swd_, Print& output_) : swd(swd_), output(output_), currentAP(0xFF), currentBank(0)
    {
    }


    template <typename T, typename V> bool exists(T& t, V value)
    {
    		return std::find(std::begin(t), std::end(t), value) != std::end(t);
    }

    /**
     * Initialize the debug port.
     */
    SWDResult initialize()
    {
    		currentAP = 0xFF;		// ensure a select request is sent
    		currentBank = 0;

    		CHECK_SUCCESS(swd.initialize());
		// "After the host has transmitted a line request sequence to the
		// SW-DP, it must read the IDCODE register." p.5-10
    		uint32_t idcode;
		//output.println("reading id code");
    		CHECK_SUCCESS(cmd(Read::IDCODE, idcode));

        if (!exists(ID_CODES, idcode))
        {
            output.printlnf("warning: unexpected idcode: %x", idcode);
            return SWDResult::ProtocolError;
        }

        clearErrors(true, true, true, true);
        uint32_t s;

        CHECK_SUCCESS(cmd(Read::CTRL_STAT, s));
        CHECK_SUCCESS(cmd(Write::CTRL_STAT, 0x50000000));
        CHECK_SUCCESS(cmd(Read::CTRL_STAT, s));

        if (((s >> 24) & 0xF0) != 0xF0)
        {
            output.printlnf("Error powering up subsystems, status: %x", s);
            return SWDResult::InitializationError;
        }

        return SWDResult::Ok;
    }

    SWDResult clearErrors(bool orunerr, bool wdataerr, bool stickyerr, bool stickycmp, bool dap=false)
    {
        uint32_t value = 0x00000000;
        if (orunerr)
        		value |= 0x10;
        if (wdataerr)
        		value |= 0x08;
        if (stickyerr)
        		value |= 0x04;
        if (stickycmp)
        		value |= 0x02;
        if (dap)
        		value |= 0x01;
        return cmd(Write::ABORT, value);
    }

    /**
     * Sets values in the control register
     * @param trnCount	The current transaction count
     * @param trnMode	The number of cycles in turnaround. Currently only 0 is supported meaning 1 cycle.
     * @param maskLane
     * @param overrunDetect	Enable overrun detection. Presently not supported.
     */
    void control(uint16_t trnCount=0, uint8_t trnMode=0, uint8_t maskLane=0, bool overrunDetect=false)
    {
        uint32_t value = 0x54000000;
        value |= ((trnCount & 0xFFF) << 12);
        value |= ((maskLane & 0x00F) << 8);
        value |= ((trnMode  & 0x003) << 2);
        if (overrunDetect)
        		value |= 0x01;
        cmd(Write::CTRL_STAT, value);
    }

    /**
     * Read from a DP register.
     */
    inline SWDResult cmd(Read reg, uint32_t& data)
    {
    		return swd.read(Port::DP, static_cast<int>(reg), data);
    }

    /**
     * Write to a DP register.
     */
    inline SWDResult cmd(Write reg, uint32_t data)
    {
    		return swd.write(Port::DP, static_cast<int>(reg), data);
    }

    /**
     * Select an AP peripheral and register bank.
     */
    SWDResult select(uint16_t apsel, uint8_t apbank, bool force=false)
    {
    		SWDResult error = SWDResult::Ok;
        if (force || apsel != this->currentAP || apbank != this->currentBank)
        {
			uint32_t value =((apsel  & 0xFF) << 24) | ((apbank & 0x0F) << 4);
			error = cmd(Write::SELECT, value);
			if (!failed(error))
			{
				this->currentAP = apsel;
				this->currentBank = apbank;
			}
        }
        return error;
    }

    /**
     * Single-transaction read AP.
     * Callers should be aware that reads are buffered and delayed by one read.
     * @param apsel The ID of the AP to select
     * @param address	The address of the register in the AP to read.
     */
    SWDResult readAP(uint8_t apsel, uint8_t address, uint32_t& data)
    {
        uint8_t adrBank = (address >> 4) & 0xF;
        uint8_t adrReg = address & 0xC;
        //output.printlnf("readAP %d %d %d", apsel, adrBank, adrReg);
		this->select(apsel, adrBank);
        return swd.read(Port::AP, adrReg<<1, data);
    }

    /**
     * Single-transaction write AP.
     */
    SWDResult writeAP(uint8_t apsel, uint8_t address, uint32_t data)
    {
        uint8_t adrBank = (address >> 4) & 0xF;
        uint8_t adrReg = address & 0xC;
		this->select(apsel, adrBank);
		return this->swd.write(Port::AP, adrReg<<1, data);
    }
};

/**
 * @param Read	Type of registers that can be read. Must be statically convertible to a uint8_t.
 * @param Write	Type of registers that can be written. Must be statically convertible to a uint8_t.
 */
template <typename Read, typename Write>
class AccessPort
{
	DebugPort& dp;

	const uint8_t apsel;


public:

	AccessPort(DebugPort& debugPort, uint8_t apselect) : dp(debugPort), apsel(apselect)
	{
	}

	SWDResult initialize()
	{
		return dp.initialize();
	}

    inline SWDResult readBuffer(uint32_t& value)
    {
    		return dp.cmd(DebugPort::Read::READBUFF, value);
    }


    inline SWDResult singleRead(Read reg, uint32_t& data)
    {
    		// the first byte read is not valid
    		CHECK_SUCCESS(cmd(reg, data));
    		return readBuffer(data);
    }

    /**
     * Reads the value from an AP register.
     */
    inline SWDResult cmd(Read reg, uint32_t& data)
    {
    		return dp.readAP(apsel, static_cast<int>(reg), data);
    }

    /**
     * Reads the value from an AP register.
     */
    inline SWDResult cmd(Write reg, uint32_t data)
    {
    		return dp.writeAP(apsel, static_cast<int>(reg), data);
    }

};

// Run a command, possibly repeating/waiting etc as needed.
#define retry(x) \
	CHECK_SUCCESS(x)

struct MEM_AP_Regs
{
	enum class Read {
		CSW = 0x00,
		TAR = 0x04,
		DRW = 0x0C,
		IDR = 0xFC
	};

	enum class Write {
		CSW = 0x00,
		TAR = 0x04,
		DRW = 0x0C
	};

};


class MEM_AP : public AccessPort<MEM_AP_Regs::Read, MEM_AP_Regs::Write>, MEM_AP_Regs
{
	const uint32_t id;

public:
	MEM_AP(DebugPort& debugPort, uint8_t apselect, uint32_t idcode) : AccessPort(debugPort, apselect), id(idcode)
	{
	}

	SWDResult initialize()
	{
		CHECK_SUCCESS(AccessPort::initialize());
		uint32_t value;
		CHECK_SUCCESS(idcode(value));
		if (value!=id)
			return SWDResult::MemAPInitializationError;

        return this->csw(1,2);
	}

	/**
	 * Sets the control status register.
	 */
    SWDResult csw(uint32_t addrInc, uint32_t size)
    {
    		uint32_t value;
    		CHECK_SUCCESS(singleRead(Read::CSW, value));
        CHECK_SUCCESS(cmd(Write::CSW, (value & 0xFFFFFF00) | (addrInc << 4) | size));
		return SWDResult::Ok;
    }

    SWDResult idcode(uint32_t& result)
    {
    		CHECK_SUCCESS(singleRead(Read::IDR, result));
    		return SWDResult::Ok;
    }

    SWDResult readWord(uint32_t adr, uint32_t& value)
    {
    		CHECK_SUCCESS(cmd(Write::TAR, adr));
    		CHECK_SUCCESS(singleRead(Read::DRW, value));
    		return SWDResult::Ok;
    }

    SWDResult writeWord(uint32_t adr, uint32_t data)
    {
		CHECK_SUCCESS(cmd(Write::TAR, adr));
		CHECK_SUCCESS(cmd(Write::DRW, data));
		return SWDResult::Ok;
    }

    SWDResult writeBlock(uint32_t adr, uint32_t count, const uint32_t* data)
    {
    		// set up the initial address
    		CHECK_SUCCESS(cmd(Write::TAR, adr));
    		while (count-->0)
    		{
    			CHECK_SUCCESS(cmd(Write::DRW, *data++));
    		}
    		return SWDResult::Ok;
    }

    /*
    void readBlock (uint32_t adr, uint32_t count, vector<uint8_t>& data) {
        this->dp.writeAP(this->apsel, 0x04, adr);
    		this->db.readAP(this->apsel, 0x0C);
    		for (uint32_t c=0; c<count-1; c++) {
    			data[c] = this->db.readAP(this->apsel, 0x0C);
    		}
    		data[count-1] = this->dp.readRB();
	}

	*/

    /*
    def writeBlockNonInc (self, adr, data):
        this->csw(0, 2) // 32-bit non-incrementing addressing
        this->dp.writeAP(this->apsel, 0x04, adr)
        for val in data:
            this->dp.writeAP(this->apsel, 0x0C, val)
        this->csw(1, 2) # 32-bit auto-incrementing addressing

    def writeHalfs (self, adr, data):
        """ Write half-words """
        this->csw(2, 1) # 16-bit packed-incrementing addressing
        this->dp.writeAP(this->apsel, 0x04, adr)
        for val in data:
            time.sleep(0.001)
            this->dp.writeAP(this->apsel, 0x0C, val, ignore = True)
        this->csw(1, 2) # 32-bit auto-incrementing addressing
     */
};


class STM32F205
{
	MEM_AP ahb;

public:


	// when compiling on a stm32f2 device, these values are already defined
	#undef FLASH_CR_PG
	#undef FLASH_CR_SER
	#undef FLASH_CR_STRT
	#undef FLASH_CR_PSIZE_8
	#undef FLASH_CR_PSIZE_16
	#undef FLASH_CR_PSIZE_32
	#undef FLASH_CR_PSIZE_64

	#undef FLASH_OPTCR_OPTLOCK
	#undef FLASH_CR_LOCK
	#undef FLASH_OPTCR_OPTSTRT
	#undef FLASH_OPTCR_RDP_0

    const uint32_t BOOTLOADER_ADDRESS = 0x8000000;

	const uint32_t DHCSR = 0xE000EDF0;
	const uint32_t AIRCR = 0xE000ED0C;
	const uint32_t DEMCR = 0xE000EDFC;

	// value that must be written to upper 16-bits of DHCSR to gain write access
	const uint32_t DHCSR_DEBUG_KEY = 0xA05F0000;
	const uint32_t AIRCR_VECT_KEY = 0x05FA0000;

	const uint32_t flash_base = 0x40023C00;	// (AHB1PERIPH_BASE + 0x3C00)

	const uint32_t FLASH_KEYR = flash_base+0x04;
	const uint32_t FLASH_OPTKEYR = flash_base+0x08;
	const uint32_t FLASH_SR = flash_base+0x0C;
	const uint32_t FLASH_CR = flash_base+0x10;
	const uint32_t FLASH_OPTCR = flash_base+0x14;

	const uint32_t FLASH_CR_LOCK = 1<<31;

	const uint32_t FLASH_SR_BUSY = 1<<16;

	/**
	 * Programming error flags. Cleared by writing this value into FLASH_CR.
	 */
	const uint32_t FLASH_SR_ERROR_FLAGS = 0xF0;

	/**
	 * Locks the FLASH_OPTCR register.
	 */
	const uint32_t FLASH_OPTCR_OPTLOCK = 1<<0;
	const uint32_t FLASH_OPTCR_OPTSTRT = 1<<1;
	const uint32_t FLASH_OPTCR_RDP_MASK = 0xFF << 8;
	const uint32_t FLASH_OPTCR_RDP_0 = 0xAA << 8;

	STM32F205(DebugPort& debugPort) : ahb(debugPort, 0, 	0x24770011) {}

	SWDResult initialize()
	{
		return ahb.initialize();
	}

	inline SWDResult write(uint32_t address, uint32_t value)
	{
		return ahb.writeWord(address, value);
	}

	inline SWDResult read(uint32_t address, uint32_t& value)
	{
		return ahb.readWord(address, value);
	}

	/**
	 * Halt the MCU
	 */
    SWDResult halt()
    {
        return write(DHCSR, DHCSR_DEBUG_KEY+0x0003 /* STEP AND HALT */);
    }

    SWDResult unhalt()
    {
        return write(DHCSR, DHCSR_DEBUG_KEY+0x0000);
    }

    SWDResult reset()
    {
        // restart the processor and peripherals
        return write(AIRCR, AIRCR_VECT_KEY+0x0004);
    }

    SWDResult flashControlLock()
    {
    		uint32_t ctrl = 0;
    		CHECK_SUCCESS(read(FLASH_CR, ctrl));
    		if (!(ctrl & FLASH_CR_LOCK))
    		{
    			CHECK_SUCCESS(write(FLASH_CR, FLASH_CR_LOCK));
    		}
    		return SWDResult::Ok;
    }

    SWDResult flashControlUnlock()
    {
		uint32_t ctrl;
		CHECK_SUCCESS(read(FLASH_CR, ctrl));
		if (ctrl & FLASH_CR_LOCK)
		{
			// unlock the flash control register by writing the magic values to the key register
			CHECK_SUCCESS(write(FLASH_KEYR, 0x45670123));
			CHECK_SUCCESS(write(FLASH_KEYR, 0xCDEF89AB));
			CHECK_SUCCESS(read(FLASH_CR, ctrl));
			if (ctrl & FLASH_CR_LOCK)
				return SWDResult::OperationError;
    		}
		return SWDResult::Ok;
    }

    SWDResult flashOptionsUnlock()
    {
    		CHECK_SUCCESS(write(FLASH_OPTKEYR, 0x08192A3B));
    		CHECK_SUCCESS(write(FLASH_OPTKEYR, 0x4C5D6E7F));
    		return SWDResult::Ok;
    }

    SWDResult flashOptionsLock()
    {
    		uint32_t value;
    		CHECK_SUCCESS(read(FLASH_OPTCR, value));
    		value |= FLASH_OPTCR_OPTLOCK;
    		return writeOptCR(value);
    }

    SWDResult writeOptCR(uint32_t value)
    {
    		CHECK_SUCCESS(flashWaitReady());
    		CHECK_SUCCESS(write(FLASH_OPTCR, value));
    		CHECK_SUCCESS(flashWaitReady());
    		return SWDResult::Ok;
    }


    uint32_t FLASH_nWRP_SECTOR_0 = 1<<16;
    /**
     * Removes the write protection on the first sector.
     */
    SWDResult programUnlock()
    {
    		uint32_t current;
    		CHECK_SUCCESS(read(FLASH_OPTCR, current));

		current |= FLASH_nWRP_SECTOR_0;							// assert no write protect on sector 0
		current |= FLASH_OPTCR_OPTSTRT;

		CHECK_SUCCESS(writeOptCR(current));
		CHECK_SUCCESS(read(FLASH_OPTCR, current));
		if (!(current & FLASH_nWRP_SECTOR_0))
			return SWDResult::OperationError;
		return SWDResult::Ok;
    }

    /**
     * Locks the first sector by enabling write protection.
     */
    SWDResult programLock()
    {
		uint32_t current;
		CHECK_SUCCESS(read(FLASH_OPTCR, current));

		current &= ~FLASH_nWRP_SECTOR_0;			// clear no write protect on sector 0
		current |= FLASH_OPTCR_OPTSTRT;

		CHECK_SUCCESS(write(FLASH_OPTCR, current));
		return SWDResult::Ok;
    }

    /**
     * Writes an image to the first sector.
     * @param data 	The binary data for the bootloader
     * @param len		The length of data for the bootloader. Maximum value is 16384.
     */
    SWDResult flashBootloader(const void* data, uint32_t len, bool eraseOnly=false)
    {
    		SWDResult result = flashBegin();
    		if (!failed(result))
    		{
    			result = flashErase();
    			if (!failed(result) && !eraseOnly)
    				result = flashImage((const uint32_t*)data, (len+3)>>2);
    		}
    		SWDResult cleanup = flashEnd();
    		return failed(result) ? result : cleanup;
    }

    /**
     * Writes an image to the first sector.
     * @param data 	The binary data for the bootloader
     * @param len		The length of data for the bootloader. Maximum value is 16384.
     */
    SWDResult eraseBootloader()
    {
    		return flashBootloader(nullptr, 0, true);
    }


    bool isRDPLevel0(uint32_t opts)
    	{
    		return ((opts >> 8) & 0xFF) == FLASH_OPTCR_RDP_0;
    	}

    SWDResult setRDPLevel0(uint32_t opts)
    {
    		opts &= ~FLASH_OPTCR_RDP_MASK;
    		opts |= FLASH_OPTCR_RDP_0;
    		opts |= FLASH_OPTCR_OPTSTRT;
    		return writeOptCR(opts);
    }

private:

    SWDResult flashBegin()
    {
		CHECK_SUCCESS(reset());
    		CHECK_SUCCESS(halt());
    		CHECK_SUCCESS(flashControlUnlock());
    		CHECK_SUCCESS(flashOptionsUnlock());
    		CHECK_SUCCESS(programUnlock());
    		return SWDResult::Ok;
    }

    SWDResult flashEnd()
    {
    		// reset all flash operations
    		CHECK_SUCCESS(write(FLASH_CR, 0));
    		CHECK_SUCCESS(programLock());
    		CHECK_SUCCESS(flashOptionsLock());
    		CHECK_SUCCESS(flashControlLock());
    		return SWDResult::Ok;
    }

    SWDResult flashImage(const uint32_t* data, uint32_t len)
    {
    		// set up the control register for word programming
    		CHECK_SUCCESS(write(FLASH_CR, FLASH_CR_PSIZE_32 | FLASH_CR_PG));

		CHECK_SUCCESS(flashWaitReady());
		CHECK_SUCCESS(flashData(data, len, BOOTLOADER_ADDRESS));

		unhalt();
		reset();
    		return SWDResult::Ok;
    }

    SWDResult flashData(const uint32_t* data, uint32_t len, uint32_t address)
    {
		while (len-->0)
		{
			CHECK_SUCCESS(write(address, *data++));
			CHECK_SUCCESS(flashWaitReady());
			address += 4;
		}
		return SWDResult::Ok;
    }

    SWDResult flashWaitReady(uint32_t timeout=10000)
    {
    		uint32_t status;
    		uint32_t start = millis();
    		for (;;)
    		{
    			CHECK_SUCCESS(read(FLASH_SR, status));
    			if (!(status & FLASH_SR_BUSY))
    				break;
    			if (millis()-start > timeout)
    			{
    				return SWDResult::FlashError;
    			}
    		}

    		if (status & FLASH_SR_ERROR_FLAGS)
    		{
    			// clear the errors
    			write(FLASH_SR, status & ~FLASH_SR_ERROR_FLAGS);
    		}

    		return status & FLASH_SR_ERROR_FLAGS ? SWDResult::FlashError : SWDResult::Ok;
    }

	const uint32_t FLASH_CR_PG       = (1 << 0);
	const uint32_t FLASH_CR_SER      = (1 << 1);
	const uint32_t FLASH_CR_STRT     = (1 << 16);
	const uint32_t FLASH_CR_PSIZE_8  = (0 << 8);
	const uint32_t FLASH_CR_PSIZE_16 = (1 << 8);
	const uint32_t FLASH_CR_PSIZE_32 = (2 << 8);
	const uint32_t FLASH_CR_PSIZE_64 = (3 << 8);

	inline uint32_t FLASH_CR_SNB(uint8_t sector)
	{
		return sector << 3;
	}

    SWDResult flashErase()
    {
    		// erase just the first sector
		CHECK_SUCCESS(flashWaitReady());
		CHECK_SUCCESS(write(FLASH_CR, FLASH_CR_SER | FLASH_CR_STRT | FLASH_CR_SNB(0)));
    		CHECK_SUCCESS(flashWaitReady());
    		return SWDResult::Ok;
    }



};
