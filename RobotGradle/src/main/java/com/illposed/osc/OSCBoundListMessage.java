
package com.illposed.osc;

import com.illposed.osc.utility.OSCJavaToByteArrayConverter;

import java.net.InetAddress;
import java.nio.charset.Charset;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.regex.Pattern;

/**
 * An OSC message that can be bound to a list once and then allow the list to control the data.
 *
 * An OSC <i>Message</i> is made up of
 * an <i>Address Pattern</i> (the receiver of the message)
 * and <i>Arguments</i> (the content of the message).
 *
 * @author Robert Hilton
 */
public class OSCBoundListMessage implements OSCPacket {

	/**
	 * Java regular expression pattern matching a single invalid character.
	 * The invalid characters are:
	 * ' ', '#', '*', ',', '?', '[', ']', '{', '}'
	 */
	private static final Pattern ILLEGAL_ADDRESS_CHAR
			= Pattern.compile("[ \\#\\*\\,\\?\\[\\]\\{\\}]");

	private String address;
	private List<Object> arguments;

	private static final OSCJavaToByteArrayConverter stream = new OSCJavaToByteArrayConverter();

	/**
	 * Creates an OSCMessage with an address
	 * and arguments already initialized.
	 * @param address  the recipient of this OSC message
	 * @param bindingList  the data sent to the receiver
	 */
	public OSCBoundListMessage(final String address, final List<Object> bindingList) {
		stream.setCharset(charset);
		checkAddress(address);
		this.address = address;
		if (bindingList == null) {
			this.arguments = new LinkedList<>();
		} else {
			this.arguments = bindingList;
		}
	}

	/**
	 * The receiver of this message.
	 * @return the receiver of this OSC Message
	 */
	public String getAddress() {
		return address;
	}

	/**
	 * Set the address of this message.
	 * @param address the receiver of the message
	 */
	public void setAddress(final String address) {
		checkAddress(address);
		this.address = address;
	}

	/**
	 * The arguments of this message.
	 * @return the arguments to this message
	 */
	public List<Object> getArguments() {
		return Collections.unmodifiableList(arguments);
	}

	/**
	 * Convert the address into a byte array.
	 * Used internally only.
	 */
	private void computeAddressByteArray() {
		stream.write(address);
	}

	/**
	 * Convert the arguments into a byte array.
	 * Used internally only.
	 */
	private void computeArgumentsByteArray() {
		stream.write(',');
		stream.writeTypes(arguments);
		for (final Object argument : arguments) {
			stream.write(argument);
		}
	}

	protected byte[] computeByteArray() {
		computeAddressByteArray();
		computeArgumentsByteArray();
		return stream.toByteArray();
	}

	/**
	 * Throws an exception if the given address is invalid.
	 * We explicitly allow <code>null</code> here,
	 * because we want to allow to set the address in a lazy fashion.
	 * @param address to be checked for validity
	 */
	private static void checkAddress(final String address) {
		// NOTE We explicitly allow <code>null</code> here,
		//   because we want to allow to set in a lazy fashion.
		if ((address != null) && !isValidAddress(address)) {
			throw new IllegalArgumentException("Not a valid OSC address: " + address);
		}
	}

	/**
	 * Checks whether a given string is a valid OSC <i>Address Pattern</i>.
	 * @param address to be checked for validity
	 * @return true if the supplied string constitutes a valid OSC address
	 */
	public static boolean isValidAddress(final String address) {
		return (address != null)
				&& !address.isEmpty()
				&& address.charAt(0) == '/'
				&& !address.contains("//")
				&& !ILLEGAL_ADDRESS_CHAR.matcher(address).find();
	}


	/** Used to encode message addresses and string parameters. */
	private Charset charset = Charset.defaultCharset();
	private InetAddress ipAddress = null;

	@Override
	public Charset getCharset() {
		return charset;
	}

	@Override
	public void setCharset(Charset charset) {
		this.charset = charset;
	}

	@Override
	public InetAddress getIPAddress() {
		return ipAddress;
	}

	@Override
	public void setIPAddress(InetAddress ipAddress) {
		this.ipAddress = ipAddress;
	}

	@Override
	public byte[] getByteArray() {
		stream.reset();
		return computeByteArray();
	}
}
