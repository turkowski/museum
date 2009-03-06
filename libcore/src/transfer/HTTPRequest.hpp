/*  Sirikata Network Utilities
 *  HTTPRequest.hpp
 *
 *  Copyright (c) 2008, Patrick Reiter Horn
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of Sirikata nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*  Created on: Jan 7, 2009 */

#ifndef SIRIKATA_HTTPRequest_HPP__
#define SIRIKATA_HTTPRequest_HPP__

#include "URI.hpp"
#include "TransferData.hpp"

namespace Sirikata {
namespace Transfer {

typedef void CURL;

typedef std::tr1::shared_ptr<class HTTPRequest> HTTPRequestPtr;

/// Downloads the specified file in another thread and calls callback when finished.
class HTTPRequest {
public:

	typedef std::tr1::function<void(HTTPRequest*,
			const DenseDataPtr &, bool)> CallbackFunc;
private:
	HTTPRequestPtr mPreventDeletion; ///< set to shared_from_this while cURL owns a reference.

	const URI mURI;
	Range mRequestedRange;
	CallbackFunc mCallback;
	CURL *mCurlRequest;
	void *mHeaders; // CURL header linked list.

	void *mCurlFormBegin;
	void *mCurlFormEnd;
	long mStatusCode;

	Range::base_type mOffset;
	DenseDataPtr mData;

	Range::base_type mUploadOffset;
	SparseData mUploadData;

	/** The default callback--useful for POST queries where you do not care about the response */
	static void nullCallback(HTTPRequest*, const DenseDataPtr &, bool){
	}

	size_t write(const unsigned char *begin, size_t amount);
	size_t read(unsigned char *begin, size_t amount);
	void gotHeader(const std::string &header);

	static void curlLoop();
	static void initCurl();
	static void destroyCurl();
	static CURL *allocDefaultCurl();

	static size_t write_cb(unsigned char *data, size_t length, size_t count, HTTPRequest *handle);
	static size_t read_cb(unsigned char *data, size_t length, size_t count, HTTPRequest *handle);
	static size_t header_cb(char *data, size_t length, size_t count, HTTPRequest *handle);

	void setFinalProperties(); ///< Will be called if the request must be retried.
	void initCurlHandle(); ///< Only called initially--sets defaults for all properties.

	HTTPRequest(const HTTPRequest &other);
public:

	/** Do not ever use this--it is not thread safe, and the DenseDataPtr
	 is passed to the callback anyway. May be made private */
	inline const DenseDataPtr &getData() {
		return mData;
	}

	/** Do not ever use this--it is not thread safe, and the DenseDataPtr
	 is passed to the callback anyway. May be made private */
	inline const SparseData &getUploadData() {
		return mUploadData;
	}

	inline Range::base_type getUploadOffset() const {
		return mUploadOffset;
	}

	/** Gets the current transfer offset--useful for a progress indicator.
	 * You can pair this up with getRange().endbyte() to get a percentage.
	 *
	 * @returns the last (absolute) byte written;
	 *          subtract getData().startbyte() to get the relative byte.
	 */
	inline Range::base_type getOffset() const {
		return mOffset;
	}

	/** Changes the location in the DenseData that we write
	 * to when we get data. Do not call this except from a callback
	 * when streaming. In most cases it will be unnecessary,
	 * as the denseData should be cleared after each packet.
	 */
	inline void seek(Range::base_type offset) {
		if (offset < mUploadData.endbyte()) {
			mUploadOffset = offset;
		} else {
			mUploadOffset = mUploadData.endbyte();
		}
	}

	HTTPRequest(const URI &uri, const Range &range)
		: mURI(uri), mRequestedRange(range), mCallback(&nullCallback),
		  mCurlRequest(NULL), mHeaders(NULL),
		  mCurlFormBegin(NULL), mCurlFormEnd(NULL)
		  {
		initCurlHandle();
	}

	~HTTPRequest();

	/// URI getter
	inline const URI &getURI() const {return mURI;}

	/// Range getter
	inline const Range &getRange() const {return mRequestedRange;}

	/** Aborts an active transfer. Necessary for some streaming applications.
	 * Acts as if the transfer failed--calls callback and removes the
	 * self-reference so this class can be free'd.
	 */
	void abort();

	void setOptions();

	/// Setter for the response function.
	inline void setCallback(const CallbackFunc &cb) {
		mCallback = cb;
	}

	/**
	 * Deletes the request file. Retrieved data may be empty,
	 * but success should be true if the deletion was successful.
	 * Should work for HTTP(s), FTP and SFTP
	 */
	void setDELETE();

	/**
	 * Performs an upload of this file. Again, there may be no retrieved data,
	 * but success should be true if the upload was successful.
	 *
	 * @param uploadData  A SparseData that represents the file.
	 *                    Any non-contiguous chunks will be filled with 0's.
	 */
	void setPUTData(const SparseData &uploadData);

	/**
	 * Performs an upload of this file using POST.
	 * Currently, only one file can be uploaded at a time,
	 * however this is relatively easy to change if necessary.
	 *
	 * This may be used in conjunction with addPOSTField if other
	 * non-file form fields are to be added.
	 *
	 * Currently, the Content-Type is hardcoded as application/octet-stream.
	 *
	 * @param fieldname  The name of the type="file" form field.
	 * @param filename   The name of the file to be uploaded.
	 * @param uploadData  A SparseData that represents the file.
	 *                    Any non-contiguous chunks will be filled with 0's.
	 */
	void setPOSTData(const std::string &fieldname,
			const std::string &filename,
			const SparseData &uploadData);

	/**
	 * Creates a POST request using the multipart/form-data enctype.
	 * This may be used in conjunction with setPOSTData, or without.
	 *
	 * @param name  The name of this form field.
	 * @param value The value of this form field.
	 */
	void addPOSTField(const std::string &name, const std::string &value);

	/** Note: you are responsible for checking the protocol
	 * and using HTTP headers in the case of http, and valid FTP
	 * commands in the case of FTP. Not doing so may cause a security bug.
	 */
	void addHeader(const std::string &header);

	/**
	 *  Executes the query.
	 *
	 *  @param holdreference If this object was constructed with a shared_ptr,
	 *      which it should have been, pass that pointer into holdReference.
	 *      If you intend to manage memory yourself (this is dangerous), it
	 *      is wise to pass NULL here.
	 */
	void go(const HTTPRequestPtr &holdReference);
};

}
}

#endif /* SIRIKATA_HTTPRequest_HPP__ */
