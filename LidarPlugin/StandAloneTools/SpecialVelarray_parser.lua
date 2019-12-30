-- trivial protocol example
-- declare our protocol
trivial_proto = Proto("SpecialVelarray","Special Velarray")
-- create a function to dissect it
function trivial_proto.dissector(buffer,pinfo,tree)
	pinfo.cols.protocol = "SpecialVelarray"
	local subtree = tree:add(trivial_proto,buffer(),"Special Velarray data")

	local curr = 0

	-- compute the number of firing return in the packet
	local headerSize = 20
	local firingSize = 8
	local footerSize = 4
	local totalSize = buffer:len()
	local totalFiringsSize = totalSize - headerSize - footerSize
	local nbFirings = totalFiringsSize / firingSize


	-- Packet Header --
	local header_subtree = subtree:add(buffer(curr,headerSize),"Header")

	header_subtree:add("Number of Firings (computed):" .. nbFirings)

	local VER_HLEN = buffer(curr,1)
	local VER = VER_HLEN:bitfield(0,4)
	local HLEN = VER_HLEN:bitfield(4,4)
	header_subtree:add(buffer(curr,1), "VER   : " .. VER)
	header_subtree:add(buffer(curr,1), "HLEN  : " .. HLEN)
	curr = curr +1

	local NXHDR = buffer(curr, 1):uint()
	header_subtree:add(buffer(curr,1),"NXHDR : " .. NXHDR)
	curr = curr +1

	local PTYPE_TLEN = buffer(curr,1)
	local PTYPE = PTYPE_TLEN:bitfield(0,4)
	local TLEN = PTYPE_TLEN:bitfield(4,4)
	header_subtree:add(buffer(curr,1), "PTYPE : " .. PTYPE)
	header_subtree:add(buffer(curr,1), "TLEN  : " .. TLEN)
	curr = curr +1

	local MIC = buffer(curr, 1):uint()
	header_subtree:add(buffer(curr,1),"MIC   : " .. MIC)
	curr = curr +1

	local PSEQ = buffer(curr, 4):uint()
	header_subtree:add(buffer(curr,4),"PSEQ  : " .. PSEQ)
	curr = curr +4

	local TREF = buffer(curr, 8):uint64()
	header_subtree:add(buffer(curr,4),"TREF  : " .. TREF)
	curr = curr +8

	local GLEN_FLEN = buffer(curr,1)
	local GLEN = GLEN_FLEN:bitfield(0,4)
	local FLEN = GLEN_FLEN:bitfield(4,4)
	header_subtree:add(buffer(curr,1), "GLEN  : " .. GLEN)
	header_subtree:add(buffer(curr,1), "FLEN  : " .. FLEN)
	curr = curr +1

	local DSET = buffer(curr, 1):uint()
	header_subtree:add(buffer(curr,1),"DSET  : " .. DSET)
	curr = curr +1

	local ISET = buffer(curr, 2):uint()
	header_subtree:add(buffer(curr,1),"ISET  : " .. ISET)
	curr = curr +2

	---- Firing Return ----
	local firingreturns = subtree:add(buffer(curr,firingSize*nbFirings),"Firing returns")

	for i=0, nbFirings-1
	do
		local firingreturn_subtree = firingreturns:add(buffer(curr,firingSize),"Firing Return : " ..i)

		HDIR_VDIR_VDFL = buffer(curr,2)
		local HDIR = HDIR_VDIR_VDFL:bitfield(0,1)
		local VDIR = HDIR_VDIR_VDFL:bitfield(1,1)
		local VDFL = HDIR_VDIR_VDFL:bitfield(2,14)
		firingreturn_subtree:add(buffer(curr,1), "HDIR : " .. HDIR)
		firingreturn_subtree:add(buffer(curr,1), "VDIR : " .. VDIR)
		firingreturn_subtree:add(buffer(curr,2), "VDFL : " .. VDFL)
		curr = curr+2

		local AZM = buffer(curr,2):uint()
		firingreturn_subtree:add(buffer(curr,2),"AZM  : " .. AZM)
		curr = curr+2

		local DIST = buffer(curr,2):uint()
		firingreturn_subtree:add(buffer(curr,2),"DIST : " .. DIST)
		curr = curr+2

		local RFT = buffer(curr,1):uint()
		firingreturn_subtree:add(buffer(curr,1),"RFT  : " .. RFT)
		curr = curr+1

		local LCN = buffer(curr,1):uint()
		firingreturn_subtree:add(buffer(curr,1),"LCN  : " .. LCN)
		curr = curr+1
	end

	-- Packet Footer --
	local footer_subtree = subtree:add(buffer(curr,footerSize),"Footer")

	local CRC = buffer(curr, 2):uint()
	footer_subtree:add(buffer(curr,2),"CRC   : " .. CRC)
	curr = curr +2

	local AC = buffer(curr,1):uint()
	footer_subtree:add(buffer(curr,1),"AC    : " .. AC)
	curr = curr+1

	local PSEQF = buffer(curr,1):uint()
	footer_subtree:add(buffer(curr,1),"PSEQF : " .. PSEQF)
	curr = curr+1

end


-- load the udp.port table
udp_table = DissectorTable.get("udp.port")
-- register our protocol to handle udp port 7777
udp_table:add(2368,trivial_proto)
