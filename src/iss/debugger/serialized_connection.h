/*******************************************************************************
 * Copyright (C) 2017, 2018, MINRES Technologies GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Contributors:
 *       eyck@minres.com - initial API and implementation
 ******************************************************************************/

#ifndef _SERIALIZED_CONNECTION_H_
#define _SERIALIZED_CONNECTION_H_

#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>

#ifdef USE_TEXT
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
using oarchive_type = boost::archive::text_oarchive;
typedef boost::archive::text_iarchive iarchive_type;
#else
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
using oarchive_type = boost::archive::binary_oarchive;
using iarchive_type = boost::archive::binary_iarchive;
#endif
#include <boost/bind/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tuple/tuple.hpp>
#include <iomanip>
#include <sstream>
#include <string>
#include <util/logging.h>
#include <vector>

/// The connection class provides serialization primitives on top of a socket.
/**
 * Each message sent using this class consists of:
 * @li An 8-byte header containing the length of the serialized data in
 * hexadecimal.
 * @li The serialized data.
 */
template <typename TREQ, typename TRESP>
class connection : public boost::enable_shared_from_this<connection<TREQ, TRESP>> {
public:
    using ptr = boost::shared_ptr<connection<TREQ, TRESP>>;

    class async_listener : public boost::enable_shared_from_this<async_listener> {
    public:
        virtual void send_completed(const boost::system::error_code &error) = 0;
        virtual void receive_completed(const boost::system::error_code &error, TREQ *result) = 0;
    };
    /// Constructor.
    connection(boost::asio::io_service &io_service)
    : socket_(io_service) {}
    /// Get the underlying socket. Used for making a connection or for accepting
    /// an incoming connection.
    boost::asio::ip::tcp::socket &socket() { return socket_; }
    ///
    void add_listener(boost::shared_ptr<async_listener> l) { listener = l; }
    ///
    void remove_listener() { listener.reset(); }
    /// Asynchronously write a data structure to the socket.
    void async_write(TRESP &t) { async_write(&t); }
    /// Asynchronously write a data structure to the socket.
    void async_write(TRESP *t) {
        // Serialize the data first so we know how large it is.
        std::ostringstream archive_stream;
        oarchive_type archive(archive_stream);
        archive << t;
        outbound_data_ = archive_stream.str();
#ifdef EXTENDED_TRACE
        CLOG(TRACE, connection) << "outbound async data with len " << outbound_data_.size() << ":'" << outbound_data_
                                << "'";
#endif
        // Format the header.
        std::ostringstream header_stream;
        header_stream << std::setw(header_length) << std::hex << std::setfill('0') << outbound_data_.size();
        if (!header_stream || header_stream.str().size() != header_length) {
            // Something went wrong, inform the caller.
            boost::system::error_code err(boost::asio::error::invalid_argument);
            if (listener) listener->send_completed(err);
            return;
        }
        outbound_header_ = header_stream.str();
        // Write the serialized data to the socket. We use "gather-write" to send
        // both the header and the data in a single write operation.
        std::vector<boost::asio::const_buffer> buffers;
        buffers.push_back(boost::asio::buffer(outbound_header_));
        buffers.push_back(boost::asio::buffer(outbound_data_));
        boost::asio::async_write(socket_, buffers,
                                 boost::bind(&connection::handle_async_write, this->shared_from_this(),
                                             boost::asio::placeholders::error,
                                             boost::asio::placeholders::bytes_transferred));
    }

protected:
    /// notify the listener about the finish of the send process
    void handle_async_write(const boost::system::error_code &err, size_t /*bytes_transferred*/) {
        if (listener != NULL) listener->send_completed(err);
    }

public:
    /// Asynchronously read a data structure from the socket.
    void async_read() {
        // Issue a read operation to read exactly the number of bytes in a header.
        boost::asio::async_read(socket_, boost::asio::buffer(inbound_header_),
                                boost::bind(&connection::async_read_header, this, boost::asio::placeholders::error));
    }

protected:
    /// Handle a completed read of a message header. The handler is passed using
    /// a tuple since boost::bind seems to have trouble binding a function object
    /// created using boost::bind as a parameter.
    void async_read_header(const boost::system::error_code &e) {
        if (e) {
            if (listener) listener->receive_completed(e, NULL);
        } else {
            // Determine the length of the serialized data.
            std::istringstream is(std::string(inbound_header_, header_length));
            std::size_t inbound_data_size = 0;
            if (!(is >> std::hex >> inbound_data_size)) {
                // Header doesn't seem to be valid. Inform the caller.
                boost::system::error_code error(boost::asio::error::invalid_argument);
                if (listener) listener->receive_completed(e, NULL);
                return;
            }
            // Start an asynchronous call to receive the data.
            inbound_data_.resize(inbound_data_size);
            boost::asio::async_read(socket_, boost::asio::buffer(inbound_data_),
                                    boost::bind(&connection::async_read_data, this, boost::asio::placeholders::error));
        }
    }

    /// Handle a completed read of message data.
    void async_read_data(const boost::system::error_code &e) {
        if (e) {
            if (listener) listener->receive_completed(e, NULL);
        } else {
            // Extract the data structure from the data just received.
            try {
                std::string archive_data(&inbound_data_[0], inbound_data_.size());
#ifdef EXTENDED_TRACE
                CLOG(TRACE, connection) << "inbound async data with len " << inbound_data_.size() << ":'"
                                        << archive_data << "'";
#endif
                std::istringstream archive_stream(archive_data);
                iarchive_type archive(archive_stream);
                TREQ *t;
                archive >> t;
                if (listener) listener->receive_completed(e, t);
            } catch (std::exception & /* unnamed */) {
                // Unable to decode data.
                boost::system::error_code err(boost::asio::error::invalid_argument);
                if (listener) listener->receive_completed(err, NULL);
                return;
            }
        }
    }

public:
    ///
    void write_data(boost::shared_ptr<TRESP> &t) { write_data(t.get()); }

    void write_data(TRESP &t) { write_data(&t); }

    void write_data(TRESP *t) {
        boost::system::error_code ec;
        this->write_data(t, ec);
        boost::asio::detail::throw_error(ec);
    }

    void write_data(TRESP *t, boost::system::error_code &ec) {
        // Serialize the data first so we know how large it is.
        std::ostringstream archive_stream;
        oarchive_type archive(archive_stream);
        archive << t;
        outbound_data_ = archive_stream.str();
#ifdef EXTENDED_TRACE
        CLOG(TRACE, connection) << "outbound sync data with len " << outbound_data_.size() << ":'" << outbound_data_
                                << "'";
#endif // Format the header.
        std::ostringstream header_stream;
        header_stream << std::setw(header_length) << std::hex << std::setfill('0') << outbound_data_.size();
        if (!header_stream || header_stream.str().size() != header_length) {
            // Something went wrong, inform the caller.
            ec.assign(boost::asio::error::invalid_argument, boost::system::system_category());
            return;
        }
        outbound_header_ = header_stream.str();
        // Write the serialized data to the socket. We use "gather-write" to send
        // both the header and the data in a single write operation.
        std::vector<boost::asio::const_buffer> buffers;
        buffers.push_back(boost::asio::buffer(outbound_header_));
        buffers.push_back(boost::asio::buffer(outbound_data_));
        boost::asio::write(socket_, buffers);
    }

    void read_data(boost::shared_ptr<TREQ> &msg) {
        TREQ *m;
        read_data(m);
        msg.reset(m);
    }

    void read_data(TREQ *&t) {
        boost::system::error_code ec;
        this->read_data(t, ec);
        boost::asio::detail::throw_error(ec);
    }

    void read_data(TREQ *&t, boost::system::error_code &ec) {
        boost::asio::read(socket_, boost::asio::buffer(inbound_header_, header_length),
                          boost::asio::transfer_exactly(header_length));
        // Determine the length of the serialized data.
        std::istringstream is(std::string(inbound_header_, header_length));
        std::size_t inbound_data_size = 0;
        if (!(is >> std::hex >> inbound_data_size)) {
            // Header doesn't seem to be valid. Inform the caller.
            ec.assign(boost::asio::error::invalid_argument, boost::system::system_category());
            return;
        }
        // Start an synchronous call to receive the data.
        inbound_data_.resize(inbound_data_size);
        boost::asio::read(socket_, boost::asio::buffer(inbound_data_, inbound_data_size),
                          boost::asio::transfer_exactly(inbound_data_size));
        std::string archive_data(&inbound_data_[0], inbound_data_.size());
#ifdef EXTENDED_TRACE
        CLOG(TRACE, connection) << "inbound sync data with len " << inbound_data_.size() << ":'" << archive_data << "'";
#endif
        std::istringstream archive_stream(archive_data);
        iarchive_type archive(archive_stream);
        archive >> t;
        return;
    }

private:
    /// The underlying socket.
    boost::asio::ip::tcp::socket socket_;
    /// The size of a fixed length header.
    enum { header_length = 8 };
    /// Holds an outbound header.
    std::string outbound_header_;
    /// Holds the outbound data.
    std::string outbound_data_;
    /// Holds an inbound header.
    char inbound_header_[header_length];
    /// Holds the inbound data.
    std::vector<char> inbound_data_;

    boost::shared_ptr<async_listener> listener;
};

template <>
class connection<std::string, std::string>
    : public boost::enable_shared_from_this<connection<std::string, std::string>> {
public:
    class async_listener : public boost::enable_shared_from_this<async_listener> {
    public:
        virtual void send_completed(const boost::system::error_code &error) = 0;
        virtual void receive_completed(const boost::system::error_code &error, std::string *result) = 0;
        virtual bool message_completed(std::vector<char> &buffer) { return true; };
    };
    /// Constructor.
    connection(boost::asio::io_context &io_service)
    : socket_(io_service) {}
    /// Get the underlying socket. Used for making a connection or for accepting
    /// an incoming connection.
    boost::asio::ip::tcp::socket &socket() { return socket_; }
    ///
    void add_listener(boost::shared_ptr<async_listener> l) { listener = l; }
    /// template specialization for std::string to allow plain communication
    /// Asynchronously write a string to the socket.
    void async_write(const std::string &str) {
        // Serialize the data first so we know how large it is.
        outbound_data_ = str;
#ifdef EXTENDED_TRACE
        LOG(TRACE) << "outbound async data with len " << outbound_data_.size() << ":'" << outbound_data_ << "'";
#endif
        // Write the outbound data to the socket. We use "gather-write" to send
        // both the header and the data in a single write operation.
        std::vector<boost::asio::const_buffer> buffers;
        buffers.push_back(boost::asio::buffer(outbound_data_));
        boost::asio::async_write(socket_, buffers, boost::bind(&connection::handle_async_write, shared_from_this(),
                                                               boost::asio::placeholders::error,
                                                               boost::asio::placeholders::bytes_transferred));
    }

protected:
    /// notify the listener about the finish of the send process
    void handle_async_write(const boost::system::error_code &err, size_t /*bytes_transferred*/) {
        if (listener != NULL) listener->send_completed(err);
    }

public:
    /// template specialization for std::string to allow plain communication
    /// Asynchronously read a string from the socket.
    void async_read() {
        // Issue a read operation to read exactly the number of bytes in a header.
        boost::asio::async_read(socket_, boost::asio::buffer(inbound_buffer_),
                                boost::bind(&connection::async_read_data, this, boost::asio::placeholders::error));
    }

protected:
    /// template specialization for std::string to allow plain communication
    /// Handle a read of string data.
    void async_read_data(const boost::system::error_code &e) {
        if (e) {
            if (listener) listener->receive_completed(e, NULL);
        } else {
            // Extract the data structure from the data just received.
            try {
                inbound_data_.push_back(inbound_buffer_[0]);
                if (listener->message_completed(inbound_data_)) {
                    std::string receive_data(&inbound_data_[0], inbound_data_.size());
#ifdef EXTENDED_TRACE
                    LOG(TRACE) << "inbound async data with len " << inbound_data_.size() << ":'" << receive_data << "'";
#endif
                    if (listener) listener->receive_completed(e, &receive_data);
                    inbound_data_.clear();
                } else
                    boost::asio::async_read(
                        socket_, boost::asio::buffer(inbound_buffer_),
                        boost::bind(&connection::async_read_data, this, boost::asio::placeholders::error));
            } catch (std::exception & /* unnamed */) {
                // Unable to decode data.
                boost::system::error_code err(boost::asio::error::invalid_argument);
                if (listener) listener->receive_completed(err, NULL);
                return;
            }
        }
    }

public:
    ///
    void write_data(const std::string &t) {
        boost::system::error_code ec;
        this->write_data(t, ec);
        boost::asio::detail::throw_error(ec);
    }

    void write_data(const std::string &t, boost::system::error_code &ec) {
        outbound_data_ = t;
#ifdef EXTENDED_TRACE
        LOG(TRACE) << "outbound sync data with len " << outbound_data_.size() << ":'" << outbound_data_ << "'";
#endif // Format the header.
        // Write the serialized data to the socket. We use "gather-write" to send
        // both the header and the data in a single write operation.
        std::vector<boost::asio::const_buffer> buffers;
        buffers.push_back(boost::asio::buffer(outbound_data_));
        boost::asio::write(socket_, buffers);
    }

    void read_data(std::string *&t) {
        boost::system::error_code ec;
        this->read_data(t, ec);
        boost::asio::detail::throw_error(ec);
    }

    void read_data(std::string *&t, boost::system::error_code &ec) {
        boost::asio::read(socket_, boost::asio::buffer(inbound_buffer_, buffer_length));
        t = new std::string(inbound_buffer_);
        return;
    }

private:
    /// The underlying socket.
    boost::asio::ip::tcp::socket socket_;
    /// The size of a fixed length header.
    enum { buffer_length = 1 };
    /// Holds the outbound data.
    std::string outbound_data_;
    /// Holds an inbound header.
    char inbound_buffer_[buffer_length];
    /// Holds the inbound data.
    std::vector<char> inbound_data_;

    boost::shared_ptr<async_listener> listener;
};
#endif /* _SERIALIZED_CONNECTION_H_ */
